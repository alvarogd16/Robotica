//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `LaserPub.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __LaserPub_h__
#define __LaserPub_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <Laser.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompLaserPub
{

class LaserPub;
class LaserPubPrx;

}

namespace RoboCompLaserPub
{

class LaserPub : public virtual ::Ice::Object
{
public:

    using ProxyType = LaserPubPrx;

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(::std::string id, const ::Ice::Current& current) const override;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current& current) const override;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual ::std::string ice_id(const ::Ice::Current& current) const override;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual void pushLaserData(::RoboCompLaser::TLaserData laserData, const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_pushLaserData(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompLaserPub
{

class LaserPubPrx : public virtual ::Ice::Proxy<LaserPubPrx, ::Ice::ObjectPrx>
{
public:

    void pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        _makePromiseOutgoing<void>(true, this, &LaserPubPrx::_iceI_pushLaserData, laserData, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto pushLaserDataAsync(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<void>>().get_future())
    {
        return _makePromiseOutgoing<void, P>(false, this, &LaserPubPrx::_iceI_pushLaserData, laserData, context);
    }

    ::std::function<void()>
    pushLaserDataAsync(const ::RoboCompLaser::TLaserData& laserData,
                       ::std::function<void()> response,
                       ::std::function<void(::std::exception_ptr)> ex = nullptr,
                       ::std::function<void(bool)> sent = nullptr,
                       const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<void>(response, ex, sent, this, &RoboCompLaserPub::LaserPubPrx::_iceI_pushLaserData, laserData, context);
    }

    /// \cond INTERNAL
    void _iceI_pushLaserData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<void>>&, const ::RoboCompLaser::TLaserData&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    LaserPubPrx() = default;
    friend ::std::shared_ptr<LaserPubPrx> IceInternal::createProxy<LaserPubPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompLaserPub
{

using LaserPubPtr = ::std::shared_ptr<LaserPub>;
using LaserPubPrxPtr = ::std::shared_ptr<LaserPubPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompLaserPub
{

class LaserPub;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< LaserPub>&);
::IceProxy::Ice::Object* upCast(LaserPub*);
/// \endcond

}

}

namespace RoboCompLaserPub
{

class LaserPub;
/// \cond INTERNAL
::Ice::Object* upCast(LaserPub*);
/// \endcond
typedef ::IceInternal::Handle< LaserPub> LaserPubPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLaserPub::LaserPub> LaserPubPrx;
typedef LaserPubPrx LaserPubPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(LaserPubPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompLaserPub
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 * Create a wrapper instance by calling ::RoboCompLaserPub::newCallback_LaserPub_pushLaserData.
 */
class Callback_LaserPub_pushLaserData_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_LaserPub_pushLaserData_Base> Callback_LaserPub_pushLaserDataPtr;

}

namespace IceProxy
{

namespace RoboCompLaserPub
{

class LaserPub : public virtual ::Ice::Proxy<LaserPub, ::IceProxy::Ice::Object>
{
public:

    void pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        end_pushLaserData(_iceI_begin_pushLaserData(laserData, context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_pushLaserData(laserData, context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_pushLaserData(laserData, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_pushLaserData(laserData, context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::RoboCompLaserPub::Callback_LaserPub_pushLaserDataPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_pushLaserData(laserData, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Context& context, const ::RoboCompLaserPub::Callback_LaserPub_pushLaserDataPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_pushLaserData(laserData, context, cb, cookie);
    }

    void end_pushLaserData(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_pushLaserData(const ::RoboCompLaser::TLaserData&, const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    /**
     * Obtains the Slice type ID corresponding to this interface.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:
    /// \cond INTERNAL

    virtual ::IceProxy::Ice::Object* _newInstance() const;
    /// \endcond
};

}

}

namespace RoboCompLaserPub
{

class LaserPub : public virtual ::Ice::Object
{
public:

    typedef LaserPubPrx ProxyType;
    typedef LaserPubPtr PointerType;

    virtual ~LaserPub();

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(const ::std::string& id, const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual const ::std::string& ice_id(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual void pushLaserData(const ::RoboCompLaser::TLaserData& laserData, const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_pushLaserData(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

protected:

    /// \cond STREAM
    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
inline bool operator==(const LaserPub& lhs, const LaserPub& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const LaserPub& lhs, const LaserPub& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

}
/// \endcond

namespace RoboCompLaserPub
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 * Create a wrapper instance by calling ::RoboCompLaserPub::newCallback_LaserPub_pushLaserData.
 */
template<class T>
class CallbackNC_LaserPub_pushLaserData : public Callback_LaserPub_pushLaserData_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_LaserPub_pushLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_LaserPub_pushLaserData<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_LaserPub_pushLaserData<T>(instance, 0, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_LaserPub_pushLaserData<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_LaserPub_pushLaserData<T>(instance, 0, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 * Create a wrapper instance by calling ::RoboCompLaserPub::newCallback_LaserPub_pushLaserData.
 */
template<class T, typename CT>
class Callback_LaserPub_pushLaserData : public Callback_LaserPub_pushLaserData_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_LaserPub_pushLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T, typename CT> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_LaserPub_pushLaserData<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T, typename CT> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_LaserPub_pushLaserData<T, CT>(instance, 0, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T, typename CT> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_LaserPub_pushLaserData<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompLaserPub::LaserPub::begin_pushLaserData.
 */
template<class T, typename CT> Callback_LaserPub_pushLaserDataPtr
newCallback_LaserPub_pushLaserData(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_LaserPub_pushLaserData<T, CT>(instance, 0, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif