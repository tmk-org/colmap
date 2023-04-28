#pragma once
#include <mutex>
#include <memory>
#include <utility>

namespace colmap::detail
{
//namespace detail
//{

template<typename T> class GPUHolder
{
public:
    using HoldingType = T;

    class RefCount
    {
    protected:
        RefCount(GPUHolder<HoldingType>* pholder):_pholder(pholder)
        {
            if(_pholder)
            {
                _pholder->IncrementRefCount();
            }
        }
        GPUHolder<HoldingType>* _pholder;
        RefCount()
            : RefCount(nullptr)
        {

        }
    public:
        
        ~RefCount()
        {
            if(_pholder)
            {
                _pholder->DecrementRefCount();
                _pholder=nullptr;
            }
        }
        static std::shared_ptr<RefCount> Create(GPUHolder<HoldingType>* pholder)
        {
            std::shared_ptr<RefCount> pLocal(new RefCount(pholder));
            return pLocal;
        }
    };
    using RefCountType = std::shared_ptr<RefCount>;
    GPUHolder(const GPUHolder&)=delete;
    GPUHolder& operator=(const GPUHolder&)=delete;
    static GPUHolder* Instance()
    {
        std::scoped_lock lock(_instanceMtx);
        auto globalInstance = _pGlobalInstance;
        if(globalInstance)
        {
            return globalInstance.get();
        }
        else
        {
            _pGlobalInstance.reset(new GPUHolder<HoldingType>());
            return _pGlobalInstance.get();
        }
    }

    

    template<typename Callable> std::pair<std::shared_ptr<HoldingType>,RefCountType> CreateGpuEntity(Callable callable)
    {
        std::scoped_lock lock(_creatorLock);
        auto ptr=_pGpuEntity;
        if(ptr)
        {
            return std::make_pair(ptr,RefCount::Create(this));
        }
        ptr=callable();
        if(!ptr)
        {
            return std::make_pair(decltype(ptr){},RefCountType{});
        }
        _pGpuEntity=ptr;
        return std::make_pair(ptr,RefCount::Create(this));
    }

protected:
    GPUHolder():_refCnt(0)
    {

    }

    void IncrementRefCount()
    {
        std::scoped_lock lock(_refcntLock);
        _refCnt++;
    }
    void DecrementRefCount()
    {
        std::scoped_lock lock(_refcntLock);
        if(_refCnt==0)
        {
            return;
        }
        if(_refCnt==1)
        {
            DestroySiftGpu();
        }
        _refCnt--;
    }

    void DestroySiftGpu()
    {
        std::scoped_lock lock(_creatorLock);
        _pGpuEntity.reset();
    }
    std::recursive_mutex _refcntLock,_creatorLock;
    int _refCnt;
    static std::recursive_mutex _instanceMtx;
    static std::shared_ptr<GPUHolder<T>> _pGlobalInstance;
    std::shared_ptr<T> _pGpuEntity;
};


//}
}

#define DECLARE_INSTANCE_MTX(type) \
    template<> std::recursive_mutex  colmap::detail::GPUHolder< type >::_instanceMtx{}
#define DECLARE_INSTANCE_PTR(type) \
    template<>  std::shared_ptr<colmap::detail::GPUHolder< type > > colmap::detail::GPUHolder< type >::_pGlobalInstance = std::shared_ptr<colmap::detail::GPUHolder< type >>()
#define GPU_HOLDER_REFCOUNT_TYPE_NAME(type) \
    RefCountGpuHolderType_##type
#define DECLARE_REFCOUNT_TYPE(type) \
    using GPU_HOLDER_REFCOUNT_TYPE_NAME(type)=colmap::detail::GPUHolder< type >::RefCountType

#define DECLARE_GPU_HOLDER_STATIC_MEMBERS(type) \
    DECLARE_REFCOUNT_TYPE(type); \
    DECLARE_INSTANCE_MTX(type); \
    DECLARE_INSTANCE_PTR(type)

//static const GPU_HOLDER_REFCOUNT_TYPE_NAME(int) staticInt;
//DECLARE_GPU_HOLDER_STATIC_MEMBERS(int);