#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSpacerItem>
#include <QWidget>
#include <QSizePolicy>
#include <QSlider>
#include <QLabel>
#include <QObject>
#include <QApplication>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QKeySequence>
#include <QPushButton>
#include <QSizePolicy>
#include <QLineEdit>
#include <QIntValidator>

#include "Control.h"

#include "qgst.h"
#include "asio.h"
#include "VideoWindow.h"
#include "VideoWidget.h"
#include "VideoStreamControl.h"
#include "IntexWidget.h"

struct Control::Impl {
  VideoWindow leftWindow;
  VideoWindow rightWindow;

  VideoWidget *leftVideoWidget;
  VideoWidget *rightVideoWidget;

  QSlider *bitrateSlider;
  QSlider *splitSlider;

  IntexWidget * intexWidget;

  VideoStreamControl videoControl;

  QShortcut switchWidgets_;
  QShortcut switchWindows_;
  QShortcut showNormal_;

  QWidget *leftVideoControl;
  QWidget *rightVideoControl;

  Impl(QWidget *parent)
      : leftWindow(parent), rightWindow(parent),
        leftVideoWidget(new VideoWidget), rightVideoWidget(new VideoWidget),
        bitrateSlider(new QSlider(Qt::Horizontal)),
        splitSlider(new QSlider(Qt::Horizontal)), intexWidget(new IntexWidget),
        videoControl(*leftVideoWidget, *rightVideoWidget,
                     *leftWindow.videoWidget(), *rightWindow.videoWidget()),
        switchWidgets_(tr("Ctrl+X"), parent, SLOT(switchWidgets())),
        switchWindows_(tr("Ctrl+Shift+X"), parent, SLOT(switchWindows())),
        showNormal_(tr("Esc"), parent, SLOT(showNormal())) {
    leftWindow.setWindowTitle("InTex Live Feed 0");
    rightWindow.setWindowTitle("InTex Live Feed 1");

    auto setBitrateChanged = [this](int) {
      const auto split = static_cast<unsigned>(splitSlider->value());
      const auto bitrate = static_cast<unsigned>(bitrateSlider->value());
    };
    QObject::connect(bitrateSlider, &QSlider::valueChanged, setBitrateChanged);
    QObject::connect(splitSlider, &QSlider::valueChanged, setBitrateChanged);
  }

  ~Impl() {}
  void switchWidgets() { videoControl.switchWidgets(); }
  void switchWindows() { videoControl.switchWindows(); }
  void setPort(enum VideoStreamControl::Stream side, const int port) {
    videoControl.setPort(side, port);
  }
};

template <typename Reconnect, typename Idr>
static QWidget *setupVideoControls(Reconnect &&reconnector, Idr &&idr) {
  auto portLabel = new QLabel("Port:");
  portLabel->setSizePolicy(QSizePolicy::Policy::Fixed,
                           QSizePolicy::Policy::Fixed);

  auto port = new QLineEdit;
  port->setValidator(new QIntValidator(0, 0xffff));
  port->setSizePolicy(QSizePolicy::Policy::Preferred,
                      QSizePolicy::Policy::Fixed);

  auto reconnect = new QPushButton("Reconnect");
  reconnect->setSizePolicy(QSizePolicy::Policy::Preferred,
                           QSizePolicy::Policy::Fixed);
  QObject::connect(reconnect, &QPushButton::clicked,
                   [ port, reconnector = std::move(reconnector) ] {
                     reconnector(port->text().toInt());
                   });

  auto portControls = new QFrame;
  auto portControlsLayout = new QHBoxLayout(portControls);

  portControlsLayout->addWidget(portLabel);
  portControlsLayout->addWidget(port);
  portControlsLayout->addWidget(reconnect);

  auto iFrameControls = new QWidget;
  auto iFrameControlsLayout = new QHBoxLayout(iFrameControls);
  iFrameControlsLayout->setContentsMargins(0, 0, 0, 0);

  auto iFrameLabel = new QLabel("I-Frame periodicity [s]:");
  auto iFrameEdit = new QLineEdit;
  iFrameEdit->setValidator(new QDoubleValidator(0.0, 100.0, 2));

  iFrameControlsLayout->addWidget(iFrameLabel);
  iFrameControlsLayout->addWidget(iFrameEdit);

  auto idrControls = new QWidget;
  auto idrControlsLayout = new QHBoxLayout(idrControls);
  idrControlsLayout->setContentsMargins(0, 0, 0, 0);

  auto idrLabel = new QLabel("IDR periodicity [s]:");
  auto idrEdit = new QLineEdit;
  idrEdit->setValidator(new QDoubleValidator(0.0, 100.0, 2));

  QObject::connect(
      idrEdit, &QLineEdit::textChanged,
      [idr = std::move(idr)](const QString &text) { qDebug() << text; });

  idrControlsLayout->addWidget(idrLabel);
  idrControlsLayout->addWidget(idrEdit);

  auto videoControls = new QFrame;
  videoControls->setFrameShape(QFrame::StyledPanel);
  auto videoControlsLayout = new QVBoxLayout(videoControls);

  videoControlsLayout->addWidget(portControls);
  videoControlsLayout->addWidget(iFrameControls);
  videoControlsLayout->addWidget(idrControls);

  return videoControls;
}

Control::Control(QWidget *parent)
    : QMainWindow(parent), d_(std::make_unique<Control::Impl>(this)) {
  setWindowTitle(QCoreApplication::applicationName());

  auto mainMenu = menuBar()->addMenu(tr("Menu"));
  mainMenu->addAction(tr("C&onnect"), this, SLOT(onConnect()),
                      QKeySequence::Open);
  mainMenu->addAction(tr("Disconnect"), this, SLOT(onDisconnect()),
                      QKeySequence::Close);

  mainMenu->addAction(tr("Preferences"));

  auto viewMenu = menuBar()->addMenu(tr("View"));
  auto viewAction = viewMenu->addAction(tr("Show Video Controls"), this,
                                        SLOT(showVideoControls(bool)),
                                        QKeySequence(tr("Ctrl+k")));
  viewAction->setCheckable(true);
  viewAction->setChecked(true);

  auto centralWidget = new QWidget;
  setCentralWidget(centralWidget);

  auto centralLayout = new QVBoxLayout(centralWidget);

  auto videoWidget = new QFrame;
  videoWidget->setFrameShape(QFrame::StyledPanel);
  auto videoLayout = new QHBoxLayout(videoWidget);

  auto leftVideo = new QFrame;
  leftVideo->setFrameShape(QFrame::StyledPanel);
  auto leftVideoLayout = new QVBoxLayout(leftVideo);

  leftVideoLayout->addWidget(d_->leftVideoWidget);
  leftVideoLayout->addItem(
      new QSpacerItem(0, 0, QSizePolicy::Fixed, QSizePolicy::MinimumExpanding));
  d_->leftVideoControl =
      setupVideoControls([this](int port) { this->setPort0(port); }, [] {});
  leftVideoLayout->addWidget(d_->leftVideoControl);

  auto rightVideo = new QFrame;
  rightVideo->setFrameShape(QFrame::StyledPanel);
  auto rightVideoLayout = new QVBoxLayout(rightVideo);

  rightVideoLayout->addWidget(d_->rightVideoWidget);
  rightVideoLayout->addItem(
      new QSpacerItem(0, 0, QSizePolicy::Fixed, QSizePolicy::MinimumExpanding));
  d_->rightVideoControl =
      setupVideoControls([this](int port) { this->setPort1(port); }, [] {});
  rightVideoLayout->addWidget(d_->rightVideoControl);

  videoLayout->addWidget(leftVideo);
  videoLayout->addWidget(rightVideo);

  connect(d_->leftVideoWidget, &VideoWidget::widgetClicked, &d_->leftWindow,
          &VideoWindow::show);
  connect(d_->rightVideoWidget, &VideoWidget::widgetClicked, &d_->rightWindow,
          &VideoWindow::show);

  centralLayout->addWidget(videoWidget);

  auto controlWidget = new QFrame;
  controlWidget->setFrameShape(QFrame::StyledPanel);
  auto controlLayout = new QHBoxLayout(controlWidget);

  auto bitrateLabel = new QLabel;
  bitrateLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

  d_->bitrateSlider->setMinimum(1);
  d_->bitrateSlider->setMaximum(1000);
  d_->bitrateSlider->setSingleStep(1);
  d_->bitrateSlider->setPageStep(10);
  d_->bitrateSlider->setSizePolicy(QSizePolicy::MinimumExpanding,
                                   QSizePolicy::Fixed);

  auto bitrateUpdate = [bitrateLabel](int bitrate) {
    bitrateLabel->setText(QString("Bitrate: %1 kBit/s").arg(bitrate));
  };
  connect(d_->bitrateSlider, &QSlider::sliderMoved, bitrateUpdate);
  connect(d_->bitrateSlider, &QSlider::valueChanged, bitrateUpdate);

  d_->bitrateSlider->setValue(800);
  d_->bitrateSlider->setTracking(false);

  auto splitLabel = new QLabel;
  bitrateLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

  d_->splitSlider->setMinimum(1);
  d_->splitSlider->setMaximum(100);
  d_->splitSlider->setSingleStep(1);
  d_->splitSlider->setPageStep(10);
  d_->splitSlider->setSizePolicy(QSizePolicy::MinimumExpanding,
                                 QSizePolicy::Fixed);

  auto splitUpdate = [splitLabel](int split) {
    splitLabel->setText(
        QString("Split: %1/%2")
            .arg(QString::number(split), QString::number(100 - split)));
  };
  connect(d_->splitSlider, &QSlider::sliderMoved, splitUpdate);
  connect(d_->splitSlider, &QSlider::valueChanged, splitUpdate);

  d_->splitSlider->setValue(50);
  d_->splitSlider->setTracking(false);

  controlLayout->addWidget(bitrateLabel);
  controlLayout->addWidget(d_->bitrateSlider);
  controlLayout->addWidget(splitLabel);
  controlLayout->addWidget(d_->splitSlider);

  centralLayout->addWidget(controlWidget);
  centralLayout->addWidget(d_->intexWidget);
}

Control::~Control() = default;

void Control::onConnect() {}
void Control::onDisconnect() {}
void Control::switchWidgets() { d_->switchWidgets(); }
void Control::switchWindows() { d_->switchWindows(); }
void Control::setPort0(const int port) {
  d_->setPort(VideoStreamControl::Stream::Left, port);
}
void Control::setPort1(const int port) {
  d_->setPort(VideoStreamControl::Stream::Right, port);
}
void Control::showVideoControls(bool show) {
  d_->leftVideoControl->setVisible(show);
  d_->rightVideoControl->setVisible(show);
}

void onBusMessage(const QGst::MessagePtr &message);

#pragma clang diagnostic ignored "-Wpadded"
#include "moc_Control.cpp"