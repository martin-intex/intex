// Copyright (c) 2013-2014 Sandstorm Development Group, Inc. and contributors
// Licensed under the MIT License:
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <map>
#include <chrono>
#include <stdexcept>

#include <capnp/rpc-twoparty.h>
#include <capnp/rpc.capnp.h>
#include <kj/debug.h>
#include <kj/threadlocal.h>

#include <QDebug>
#include <QTimer>

#include "async-io.h"
#include "ez-rpc.h"

#pragma clang diagnostic ignored "-Wshadow" 
#pragma clang diagnostic ignored "-Wweak-vtables"
#pragma clang diagnostic ignored "-Wgnu-statement-expression"

namespace intex {
namespace rpc {

KJ_THREADLOCAL_PTR(EzRpcContext) threadEzContext = nullptr;

using namespace capnp;
using namespace capnp::rpc;

class EzRpcContext: public kj::Refcounted {
public:
  EzRpcContext(): ioContext(intex::rpc::setupAsyncIo()) {
    threadEzContext = this;
  }

  ~EzRpcContext() noexcept {
    if(threadEzContext != this) {
      qCritical() << "EzRpcContext destroyed from different thread than it was "
                     "created.";
    }
    threadEzContext = nullptr;
  }

  kj::WaitScope& getWaitScope() {
    return ioContext.waitScope;
  }

  kj::AsyncIoProvider& getIoProvider() {
    return *ioContext.provider;
  }

  kj::LowLevelAsyncIoProvider& getLowLevelIoProvider() {
    return *ioContext.lowLevelProvider;
  }

  static kj::Own<EzRpcContext> getThreadLocal() {
    EzRpcContext* existing = threadEzContext;
    if (existing != nullptr) {
      return kj::addRef(*existing);
    } else {
      return kj::refcounted<EzRpcContext>();
    }
  }

private:
  QtAsyncIoContext ioContext;
};

// =======================================================================================

static kj::Promise<kj::Own<kj::AsyncIoStream>>
connectAttach(kj::Own<kj::NetworkAddress> &&addr) {
  return addr->connect().attach(kj::mv(addr));
}

struct EzRpcClient::Impl : public QObject {
  Q_OBJECT

public:
  kj::StringPtr serverAddress_;
  uint defaultPort_;
  ReaderOptions readerOpts_;
  kj::Own<EzRpcContext> context;

  struct ClientContext {
    kj::Own<kj::AsyncIoStream> stream;
    TwoPartyVatNetwork network;
    RpcSystem<rpc::twoparty::VatId> rpcSystem;

    ClientContext(kj::Own<kj::AsyncIoStream>&& stream, ReaderOptions readerOpts)
        : stream(kj::mv(stream)),
          network(*this->stream, rpc::twoparty::Side::CLIENT, readerOpts),
          rpcSystem(makeRpcClient(network)) {}

    Capability::Client getMain() {
      capnp::word scratch[4];
      memset(scratch, 0, sizeof(scratch));
      MallocMessageBuilder message(scratch);
      auto hostId = message.getRoot<rpc::twoparty::VatId>();
      hostId.setSide(rpc::twoparty::Side::SERVER);
      return rpcSystem.bootstrap(hostId);
    }

    Capability::Client restore(kj::StringPtr name) {
      capnp::word scratch[64];
      memset(scratch, 0, sizeof(scratch));
      MallocMessageBuilder message(scratch);

      auto hostIdOrphan =
          message.getOrphanage().newOrphan<rpc::twoparty::VatId>();
      auto hostId = hostIdOrphan.get();
      hostId.setSide(rpc::twoparty::Side::SERVER);

      auto objectId = message.getRoot<AnyPointer>();
      objectId.setAs<Text>(name);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      return rpcSystem.restore(hostId, objectId);
#pragma GCC diagnostic pop
    }
  };

  decltype(auto) connect() {
    using namespace std::literals::chrono_literals;
    Q_EMIT disconnected();
    return context->getIoProvider()
        .getNetwork()
        .parseAddress(serverAddress_, defaultPort_)
        .then([](kj::Own<kj::NetworkAddress> &&addr) {
          return connectAttach(kj::mv(addr));
        })
        .then(
            [this](kj::Own<kj::AsyncIoStream> &&stream) {
              auto ctx = kj::heap<ClientContext>(kj::mv(stream), readerOpts_);
              ctx->network.onDisconnect()
                  .then([this]() {
                    QTimer::singleShot((1000ms).count(), this,
                                       &Impl::reconnect);
                  })
                  .detach([](auto &&exception) {
                    qDebug() << exception.getDescription().cStr();
                  });
              clientContext = kj::mv(ctx);
              Q_EMIT connected();
            },
            [this](auto &&exception) {
              qDebug() << exception.getDescription().cStr();
              QTimer::singleShot((1000ms).count(), this, &Impl::reconnect);
            })
        .fork();
  }

  void reconnect() { setupPromise = kj::heap(connect()); }

  kj::Own<kj::ForkedPromise<void>> setupPromise;
  kj::Maybe<kj::Own<ClientContext>> clientContext;
  // Filled in before `setupPromise` resolves.

  Impl(EzRpcClient *parent, kj::StringPtr serverAddress, uint defaultPort,
       ReaderOptions readerOpts)
      : serverAddress_(serverAddress), defaultPort_(defaultPort),
        readerOpts_(readerOpts), context(EzRpcContext::getThreadLocal()) {
    QObject::connect(this, &Impl::connected, parent, &EzRpcClient::connected);
    setupPromise = kj::heap<kj::ForkedPromise<void>>(connect());
  }

  ~Impl() noexcept try {
  } catch (const kj::Exception &e) {
    qCritical() << e.getDescription().cStr();
  } catch (const std::runtime_error &e) {
    qCritical() << e.what();
  } catch (...) {
    qCritical() << "Unkown error occured" << __LINE__ << __PRETTY_FUNCTION__;
  }

// clang-format off
Q_SIGNALS:
  void connected();
  void disconnected();
// clang-format on
};

EzRpcClient::EzRpcClient(kj::StringPtr serverAddress, uint defaultPort,
                         ReaderOptions readerOpts)
    : impl(kj::heap<Impl>(this, serverAddress, defaultPort, readerOpts)) {}

EzRpcClient::~EzRpcClient() noexcept try {
} catch (const kj::Exception &e) {
  qCritical() << e.getDescription().cStr();
} catch (const std::runtime_error &e) {
  qCritical() << e.what();
} catch (...) {
  qCritical() << "Unkown error occured" << __LINE__ << __PRETTY_FUNCTION__;
}

Capability::Client EzRpcClient::getMain() {
  KJ_IF_MAYBE(client, impl->clientContext) {
    return client->get()->getMain();
  } else {
    return impl->setupPromise->addBranch().then([this]() {
      return KJ_ASSERT_NONNULL(impl->clientContext)->getMain();
    });
  }
}

kj::WaitScope& EzRpcClient::getWaitScope() {
  return impl->context->getWaitScope();
}

kj::AsyncIoProvider& EzRpcClient::getIoProvider() {
  return impl->context->getIoProvider();
}

kj::LowLevelAsyncIoProvider& EzRpcClient::getLowLevelIoProvider() {
  return impl->context->getLowLevelIoProvider();
}

// =======================================================================================

struct EzRpcServer::Impl final: public SturdyRefRestorer<AnyPointer>,
                                public kj::TaskSet::ErrorHandler {
  Capability::Client mainInterface;
  kj::Own<EzRpcContext> context;

  struct ExportedCap {
    kj::String name;
    Capability::Client cap = nullptr;

    ExportedCap(kj::StringPtr name, Capability::Client cap)
        : name(kj::heapString(name)), cap(cap) {}

    ExportedCap() = default;
    ExportedCap(const ExportedCap&) = delete;
    ExportedCap(ExportedCap&&) = default;
    ExportedCap& operator=(const ExportedCap&) = delete;
    ExportedCap& operator=(ExportedCap&&) = default;
    // Make std::map happy...
  };

  std::map<kj::StringPtr, ExportedCap> exportMap;

  kj::ForkedPromise<uint> portPromise;

  kj::TaskSet tasks;

  struct ServerContext {
    kj::Own<kj::AsyncIoStream> stream;
    TwoPartyVatNetwork network;
    RpcSystem<rpc::twoparty::VatId> rpcSystem;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    ServerContext(kj::Own<kj::AsyncIoStream>&& stream, SturdyRefRestorer<AnyPointer>& restorer,
                  ReaderOptions readerOpts)
        : stream(kj::mv(stream)),
          network(*this->stream, rpc::twoparty::Side::SERVER, readerOpts),
          rpcSystem(makeRpcServer(network, restorer)) {}
#pragma GCC diagnostic pop
  };

  Impl(Capability::Client mainInterface, kj::StringPtr bindAddress, uint defaultPort,
       ReaderOptions readerOpts)
      : mainInterface(kj::mv(mainInterface)),
        context(EzRpcContext::getThreadLocal()), portPromise(nullptr), tasks(*this) {
    auto paf = kj::newPromiseAndFulfiller<uint>();
    portPromise = paf.promise.fork();

    tasks.add(context->getIoProvider().getNetwork().parseAddress(bindAddress, defaultPort)
        .then(kj::mvCapture(paf.fulfiller,
          [this, readerOpts](kj::Own<kj::PromiseFulfiller<uint>>&& portFulfiller,
                             kj::Own<kj::NetworkAddress>&& addr) {
      auto listener = addr->listen();
      portFulfiller->fulfill(listener->getPort());
      acceptLoop(kj::mv(listener), readerOpts);
    })));
  }

#pragma clang diagnostic ignored "-Wweak-vtables"
  Impl(Capability::Client mainInterface, struct sockaddr* bindAddress, uint addrSize,
       ReaderOptions readerOpts)
      : mainInterface(kj::mv(mainInterface)),
        context(EzRpcContext::getThreadLocal()), portPromise(nullptr), tasks(*this) {
    auto listener = context->getIoProvider().getNetwork()
        .getSockaddr(bindAddress, addrSize)->listen();
    portPromise = kj::Promise<uint>(listener->getPort()).fork();
    acceptLoop(kj::mv(listener), readerOpts);
  }

  Impl(Capability::Client mainInterface, int socketFd, uint port, ReaderOptions readerOpts)
      : mainInterface(kj::mv(mainInterface)),
        context(EzRpcContext::getThreadLocal()),
        portPromise(kj::Promise<uint>(port).fork()),
        tasks(*this) {
    acceptLoop(context->getLowLevelIoProvider().wrapListenSocketFd(socketFd), readerOpts);
  }

  void acceptLoop(kj::Own<kj::ConnectionReceiver>&& listener, ReaderOptions readerOpts) {
    auto ptr = listener.get();
    tasks.add(ptr->accept().then(kj::mvCapture(kj::mv(listener),
        [this, readerOpts](kj::Own<kj::ConnectionReceiver>&& listener,
                           kj::Own<kj::AsyncIoStream>&& connection) {
      acceptLoop(kj::mv(listener), readerOpts);

      auto server = kj::heap<ServerContext>(kj::mv(connection), *this, readerOpts);

      // Arrange to destroy the server context when all references are gone, or when the
      // EzRpcServer is destroyed (which will destroy the TaskSet).
      tasks.add(server->network.onDisconnect().attach(kj::mv(server)));
    })));
  }

  Capability::Client restore(AnyPointer::Reader objectId) override {
    if (objectId.isNull()) {
      return mainInterface;
    } else {
      auto name = objectId.getAs<Text>();
      auto iter = exportMap.find(name);
      if (iter == exportMap.end()) {
        KJ_FAIL_REQUIRE("Server exports no such capability.", name) { break; }
        return nullptr;
      } else {
        return iter->second.cap;
      }
    }
  }

  void taskFailed(kj::Exception&& exception) override {
    kj::throwFatalException(kj::mv(exception));
  }
};

EzRpcServer::EzRpcServer(Capability::Client mainInterface, kj::StringPtr bindAddress,
                         uint defaultPort, ReaderOptions readerOpts)
    : impl(kj::heap<Impl>(kj::mv(mainInterface), bindAddress, defaultPort, readerOpts)) {}

EzRpcServer::~EzRpcServer() noexcept(false) {}

void EzRpcServer::exportCap(kj::StringPtr name, Capability::Client cap) {
  Impl::ExportedCap entry(kj::heapString(name), cap);
  impl->exportMap[entry.name] = kj::mv(entry);
}

kj::Promise<uint> EzRpcServer::getPort() {
  return impl->portPromise.addBranch();
}

kj::WaitScope& EzRpcServer::getWaitScope() {
  return impl->context->getWaitScope();
}

kj::AsyncIoProvider& EzRpcServer::getIoProvider() {
  return impl->context->getIoProvider();
}

kj::LowLevelAsyncIoProvider& EzRpcServer::getLowLevelIoProvider() {
  return impl->context->getLowLevelIoProvider();
}

} // namespace rpc
} // namespace intex

#pragma clang diagnostic ignored "-Wundefined-reinterpret-cast"
#include "ez-rpc.moc"
#include "moc_ez-rpc.cpp"
