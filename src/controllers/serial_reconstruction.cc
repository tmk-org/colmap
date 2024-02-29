#include "serial_reconstruction.h"

#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/option_manager.h"
#include "util/timer.h"
#include <iterator>
namespace colmap
{

bool operator==( const SerialReconstructionController::image_id_range_t& l , const SerialReconstructionController::image_id_range_t& r )
{
    if (&l == &r)
    {
        return true;
    }
    return l.first == r.first && l.second == r.second;
}

bool operator^( const SerialReconstructionController::image_id_range_t& l , const image_t& id )
{
    return id >= l.first && id <= l.second;
}

SerialReconstructionController::SerialReconstructionController(
    const OptionManager& options , ReconstructionManager* reconstruction_manager ,
    size_t max_buffer_size )
    : option_manager_( options ) ,
    reconstruction_manager_( reconstruction_manager ) ,
    database_( std::make_shared<MemoryDatabase>( ) ) ,
    reader_options_( *options.image_reader ) ,
    _max_buffer_size( max_buffer_size )
{
    CHECK_NOTNULL( reconstruction_manager_ );

    matching_queue_.reset( new JobQueue<image_t>( max_buffer_size ) );
    reader_queue_.reset( new JobQueue<internal::ImageData>( max_buffer_size ) );
    auto* pSerialSiftFeatureExtractor = new SerialSiftFeatureExtractor(
                 *option_manager_.sift_extraction ,
                 database_ ,
                 reader_queue_.get( ) );
    feature_extractor_.reset( pSerialSiftFeatureExtractor );
    feature_extractor_->AddCallback( STARTED_CALLBACK ,
                                      [ this ] ( )
                                      {
                                            if (!OnFeatureExtractionStart.empty( ))
                                            {
                                                OnFeatureExtractionStart( );
                                            }
                                      } );
    feature_extractor_->AddCallback( FINISHED_CALLBACK ,
                                      [ this ] ( )
                                      {
                                            if (!OnFeatureExtractionStop.empty( ))
                                            {
                                                OnFeatureExtractionStop( );
                                            }
                                      } );
    pSerialSiftFeatureExtractor->connectStateHandler( FeatureExtractorStateChanged );
    sequential_matcher_.reset( new SerialSequentialFeatureMatcher(
        *option_manager_.sequential_matching , *option_manager_.sift_matching ,
        database_ , matching_queue_.get( ) ) );
    sequential_matcher_->AddCallback( STARTED_CALLBACK ,
                                      [ this ] ( )
                                      {
                                            if (!OnFeatureMatchingStart.empty( ))
                                            {
                                                OnFeatureMatchingStart( );
                                            }
                                      } );
    sequential_matcher_->AddCallback( FINISHED_CALLBACK ,
                                      [ this ] ( )
                                      {
                                            if (!OnFeatureMatchingStop.empty( ))
                                            {
                                                OnFeatureMatchingStop( );
                                            }
                                      } );


    incremental_mapper_.reset( new IncrementalMapperController(
        option_manager_.mapper.get( ) , "" , database_ , reconstruction_manager_ ) );
    incremental_mapper_->AddCallback( STARTED_CALLBACK ,
                                      [ this ] ( )
                                      {
                                            if (!OnRecounstructionStart.empty( ))
                                            {
                                                OnRecounstructionStart( );
                                            }
                                      } );
    database_->Connect( [ this ] ( auto arg ) {onLoad( arg ); } );
}

void SerialReconstructionController::Stop( bool isReconstruct )
{
    reader_queue_->Wait( );
    reader_queue_->Stop( );
    LOG( INFO ) << "after stopping reader_queue_.size() " << reader_queue_->Size( );
    feature_extractor_->Wait( );
    feature_extractor_.reset( );
    reader_queue_->Clear( );
    TRACE( INFO , "feature extractor stopped" );
    matching_queue_->Wait( );
    //std::list<decltype(matching_overlap_)::key_type> keys;
    for(auto& [k, v]: matching_overlap_)
    {
        if(v>0)
        {
            matching_queue_->Push(k+1);
        }
    }
    //for (size_t i = 0; i < matching_overlap_.size( ); ++i)
    //{
    //    if (matching_overlap_[ i ] > 0)
    //    {
    //        matching_queue_->Push( i + 1 );
    //    }
    //}
    matching_queue_->Wait( );
    matching_queue_->Stop( );
    LOG( INFO ) << "after stopping matching_queue_.size() " << matching_queue_->Size( );
    sequential_matcher_->Wait( );
    sequential_matcher_.reset( );
    matching_queue_->Clear( );
    TRACE( INFO , "sequential_matcher stopped" );
    if (isReconstruct)
    {
        RunIncrementalMapper( );
    }
    LOG( INFO ) << "RunIncrementalMapper() finished";
    Thread::Stop( );
}

void SerialReconstructionController::Run( )
{
    if (IsStopped( ))
    {
        return;
    }

    RunFeatureExtraction( );

    if (IsStopped( ))
    {
        return;
    }

    RunFeatureMatching( );
}

void SerialReconstructionController::RunFeatureExtraction( )
{
    CHECK( feature_extractor_ );
    feature_extractor_->Start( );
}

void SerialReconstructionController::RunFeatureMatching( )
{
    CHECK( sequential_matcher_ );
    sequential_matcher_->Start( );
}

void SerialReconstructionController::RunIncrementalMapper( )
{
    CHECK( incremental_mapper_ );
    CheckAgainstExclude( );
    incremental_mapper_->Start( );
    LOG( INFO ) << "incremental mapper started...";
    incremental_mapper_->Wait( );
    LOG( INFO ) << "incremental mapper finished...";
    incremental_mapper_.reset( );
    
    if(!(option_manager_.project_path==nullptr || option_manager_.project_path->empty()))
    {
        const auto sparse_path = JoinPaths( *option_manager_.project_path , "sparse" );
        CreateDirIfNotExists( sparse_path );
        reconstruction_manager_->Write( sparse_path , &option_manager_ );
    }
}

void SerialReconstructionController::onLoad( image_t id )
{
    std::unique_lock lock( overlap_mutex_ );
  //  int overlap = option_manager_.sequential_matching->overlap - 1;
    int overlap = 30;
    decltype(std::declval<decltype(matching_overlap_)>().find({})) prev_it;
    decltype(std::declval<decltype(matching_overlap_)>().find({})) it;
    decltype(std::declval<decltype(matching_overlap_)>().find({})) next_it;
    prev_it = matching_overlap_.find(id -1);
    // Сheck if there are enough processed images
    if (prev_it != matching_overlap_.end() && prev_it->second == overlap)
    {
        prev_it->second = 0;
        //matching_overlap_[ id - 1 ] = 0;
        matching_queue_->Push( id );
    }

    // Update number of processed images
    for (int i = id - 2; i > int( id ) - 2 - overlap && i >= 0; --i)
    {
        it = matching_overlap_.find(i);

        if (it!=matching_overlap_.end() && ++(it->second) == overlap &&
            database_->ExistsDescriptors( i + 1 ))
        {
            it->second = 0;
            //matching_overlap_[ i ] = 0;
            while (!matching_queue_->Push( i + 1 , std::chrono::milliseconds( 100 ) ))
            {
                if (!matching_queue_->Running( ))
                {
                    break;
                }
                std::this_thread::yield( );
            }
            
        }
    }
}

void SerialReconstructionController::AddImageData(
    internal::ImageData image_data )
{
    LOGSCOPE( "image internal cam id %d internal image id %d" , image_data.image.CameraId( ) , image_data.image.ImageId( ) );
    if (!reader_queue_->Running( ))
    {
        TRACE( WARNING , "reader queue stopped,returning" );
    }
    std::list<std::tuple<std::string , int , double> > times;
    {
        std::scoped_lock lock( this->_rangesToExcludeMutex );
        if (!_rangesToExclude.empty( )
            && std::find_if( _rangesToExclude.begin( ) ,
                _rangesToExclude.end( ) ,
                [ & ] ( const auto& p_range )->bool {return p_range ^ image_data.image.ImageId( ); } ) != _rangesToExclude.end( ))
        {
            LOG( INFO ) << "frame [" << image_data.image.ImageId( ) << " , " << image_data.image.CameraId( ) << " ] skipped" << std::endl;
            return;
        }
    }
    DatabaseTransaction database_transaction( database_.get( ) );

//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    bool locked = false;
    bool reader_stopped = false;
    FINALLY(
        if (locked)
        {
            overlap_mutex_.unlock( );
        }
    );
    //std::unique_lock<std::mutex>::try_lock_for()
//        std::unique_lock<std::mutex> lock( overlap_mutex_ );
    do
    {
        locked = overlap_mutex_.try_lock_for( std::chrono::milliseconds( 100 ) );
        reader_stopped = !reader_queue_->Running( );
    }
    while (!(locked || reader_stopped) );
    if (reader_stopped)
    {
        TRACE( WARNING , "reader queue stopped,returning" );
        return;
    }
    //    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    if (cameras_ids_correspondence_.find( image_data.camera.CameraId( ) ) != cameras_ids_correspondence_.end( ))
    {
        image_data.image.SetCameraId(
            cameras_ids_correspondence_[ image_data.camera.CameraId( ) ] );
    }
    else
    {
        auto camera_id = database_->WriteCamera( image_data.camera );
        cameras_ids_correspondence_[ image_data.camera.CameraId( ) ] = camera_id;
        image_data.image.SetCameraId( camera_id );
    }
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    auto orig_image_id = image_data.image.ImageId( );
    image_data.image.SetImageId( database_->WriteImage( image_data.image ) );
    images_ids_correspondence_[ image_data.image.ImageId( ) ] = orig_image_id;
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    matching_overlap_.insert({image_data.image.ImageId(),0}) ;
  //}
    if (reader_queue_->Size( ) == _max_buffer_size)
    {
        if (!FeatureExtractorStateChanged.empty( ))
        {
            FeatureExtractorStateChanged( _max_buffer_size , 0 , 0 );
        }
    }
    LOG(INFO)   << "frame origImageId " 
                << orig_image_id
                << " internal image_id " 
                << image_data.image.ImageId() 
                <<" marked for database operations, pushing it to extractor queue";
    size_t tryouts = 0;
    while (!reader_queue_->Push( image_data , std::chrono::milliseconds( 100 ) ))
    {
        if (!reader_queue_->Running( ))
        {
            TRACE( INFO , "reader not running,push canceled" );
            break;
        }
        std::this_thread::yield( );
        if(((tryouts ++ ) % 1000)==0)
        {
            LOG(INFO) 
                << " frame internal id " 
                << image_data.image.ImageId() << " after " 
                << tryouts << " tryouts is NOT pushed to extractor input queue";  
        }
    }
    LOG(INFO)   << " frame internal id " 
                << image_data.image.ImageId() << " after " 
                << tryouts << " tryouts is pushed to extractor input queue";  
  //  timer.Pause();
  //  times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
  //  double prev_dur=0;
  //  for(auto& tt : times)
  //  {
  //    auto&[fname,line,dur] = tt;
  //    std::cout << fname <<" :"<<line<<" :"<<((dur-prev_dur)/1000.0)<<" ms"<<std::endl;
  //    prev_dur = dur;
  //  }
}

const std::unordered_map<image_t , image_t>&
SerialReconstructionController::getImageCorrespondences( ) const
{
    return images_ids_correspondence_;
}

const std::unordered_map<camera_t , camera_t>&
SerialReconstructionController::getCameraCorrespondences( ) const
{
    return cameras_ids_correspondence_;
}

void SerialReconstructionController::SetExcludeRange( const SerialReconstructionController::image_id_range_t& range )
{
    std::scoped_lock lock( _rangesToExcludeMutex );
    if (_rangesToExclude.empty( ) ||
        std::find( _rangesToExclude.begin( ) , _rangesToExclude.end( ) ,
            range ) == _rangesToExclude.end( ))
    {
        _rangesToExclude.push_back( range );
        //if(initialFrameId1 != -1 && !_rangesToExclude.empty())
        //{
        //    decltype(_rangesToExclude)::const_iterator it=std::find(  _rangesToExclude.begin(),
        //                                                    _rangesToExclude.end(),
        //                                                    [&](const auto& r)->bool{return r^initialFrameId1;} );
        //    if(it != _rangesToExclude.end())
        //    {
//
        //    }
        //}
    }
}

void SerialReconstructionController::SetExcludeRangeFromIdToEnd( image_t idfrom )
{
    SetExcludeRange( std::make_pair( idfrom , std::numeric_limits<image_t>::max( ) ) );
}

void SerialReconstructionController::SetExcludeRangeFromBeginToId( image_t idto )
{
    SetExcludeRange( std::make_pair( std::numeric_limits<image_t>::min( ) , idto ) );
}

void SerialReconstructionController::CheckAgainstExclude( )
{

    decltype( _rangesToExclude ) ranges;
    {
        std::scoped_lock lock( _rangesToExcludeMutex );
        if (_rangesToExclude.empty( ))
        {
            return;
        }
        ranges = _rangesToExclude;
    }
    DatabaseTransaction database_transaction( database_.get( ) );
    int imageNum = ( int ) ( database_->NumImages( ) & std::numeric_limits<int>::max( ) );
    std::list<image_t> imagesToRemove;
    for (int i = 0; i < imageNum; i++)
    {
        auto initial_frame_id = images_ids_correspondence_[ i ];
        if (std::find_if( ranges.begin( ) , ranges.end( ) , [ & ] ( const auto& r )->bool {return r ^ initial_frame_id; } ) != ranges.end( ))
        {
            imagesToRemove.push_back( i );
        }
    }
    imagesToRemove.sort( );
    imagesToRemove.unique( );
    std::list<std::pair<image_t , image_t>> pairs;
    image_t imageNumImaget = ( image_t ) ( imageNum );
    for (const auto& id : imagesToRemove)
    {
        for (image_t i = 0; i < imageNumImaget; i++)
        {
            if (database_->ExistsMatches( id , i ) && ( i != id ))
            {
                pairs.push_back( std::make_pair( id , i ) );
                pairs.push_back( std::make_pair( i , id ) );
            }
        }
    }
    for (auto [id1 , id2] : pairs)
    {
//        LOG( INFO ) << "removing all info for [" << id1 << "," << id2 << "] pairid " << MemoryDatabase::ImagePairToPairId( id1 , id2 ) << " before NumMatches was " << database_->NumMatches( ) << " inlier matches " << database_->NumInlierMatches( ) << " keypoints " << database_->NumKeypoints( )
//            << " exists matches " << database_->ExistsMatches( id1 , id2 )
//            << " exists inlier matches " << database_->ExistsInlierMatches( id1 , id2 )
//            << " exists keypoints " << database_->ExistsKeypoints( id1 );
        database_->DeleteMatches( id1 , id2 );
        database_->DeleteInlierMatches( id1 , id2 );
//        LOG( INFO ) << "removing all info for [" << id1 << "," << id2 << "] pairid " << MemoryDatabase::ImagePairToPairId( id1 , id2 ) << " before NumMatches was " << database_->NumMatches( ) << " inlier matches " << database_->NumInlierMatches( ) << " keypoints " << database_->NumKeypoints( )
//            << " exists matches " << database_->ExistsMatches( id1 , id2 )
//            << " exists inlier matches " << database_->ExistsInlierMatches( id1 , id2 )
//            << " exists keypoints " << database_->ExistsKeypoints( id1 );
    }
    for (const auto& id : imagesToRemove)
    {
        database_->WriteDescriptors( id , colmap::FeatureDescriptors{} );
        database_->WriteKeypoints( id , colmap::FeatureKeypoints{} );
    }
    {
        std::scoped_lock lock(this->overlap_mutex_);
        for (const auto& id : imagesToRemove)
        {
            auto it_overlap=matching_overlap_.find(id);
            if(it_overlap != matching_overlap_.end())
            {
                matching_overlap_.erase(it_overlap);
            }
            
        }
    }
    auto initial_image_id1 = this->option_manager_.mapper->init_image_id1;
    auto initial_image_id2 = this->option_manager_.mapper->init_image_id2;
    if (initial_image_id1 == -1 && initial_image_id2 == -1)
    {
        return;
    }
    std::list<image_t> newImageIdList;
    {
        auto matches = database_->ReadAllMatches( );
        for (const auto& match_pair : matches)
        {
            if (match_pair.second.size( ) == 0)
            {
                continue;
            }
            image_t id_1 , id_2;
            MemoryDatabase::PairIdToImagePair( match_pair.first , &id_1 , &id_2 );
//            LOG( INFO ) << "after garbage collection pairid " << match_pair.first << " transformed to [ " << id_1 << "," << id_2 << " ]";
            newImageIdList.push_back( id_1 );
        }
    }
    newImageIdList.sort( );
    newImageIdList.unique( );
    for (auto* p_fr_id : { &initial_image_id1,&initial_image_id2 })
    {
        auto& r_fr_id = *p_fr_id;
        if (r_fr_id == -1)
        {
            continue;
        }
        auto initial_offset = r_fr_id;
        for (auto it = newImageIdList.begin( ); it != newImageIdList.end( ); it++)
        {
            auto it_next = std::next( it );
            if (it_next == newImageIdList.end( ))
            {
                break;
            }
            if (r_fr_id <= ( int ) ( ( *it_next ) & std::numeric_limits<int>::max( ) ))
            {
                r_fr_id = *it_next;
                break;
            }
        }
        if (r_fr_id == initial_offset)
        {
            r_fr_id = *newImageIdList.begin( );
        }
    }
    if (initial_image_id1 == initial_image_id2 && initial_image_id1 != -1)
    {
        initial_image_id2 = -1;
    }
    
    LOG( INFO ) << "init_image_id1 changed from " << this->option_manager_.mapper->init_image_id1 << " to " << initial_image_id1;
    LOG( INFO ) << "init_image_id2 changed from " << this->option_manager_.mapper->init_image_id2 << " to " << initial_image_id2;
    this->option_manager_.mapper->init_image_id1 = initial_image_id1;
    this->option_manager_.mapper->init_image_id2 = initial_image_id2;

}
}  // namespace colmap
