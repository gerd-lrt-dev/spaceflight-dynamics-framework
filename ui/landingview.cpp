#include "landingview.h"

#include <QPainter>
#include <QPaintEvent>
#include <QLineF>
#include <QtMath>

LandingView::LandingView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(320, 260);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(&animationTimer, &QTimer::timeout, this, [this]() {
        motionOffset = (motionOffset + 2) % 20;
        update();
    });

    animationTimer.start(33);
}

void LandingView::setPositionENU(const Vector3& pos)
{
    positionENU = pos;

    QPointF enPoint(pos.x, pos.y);
    if (trajectoryEN.isEmpty() || QLineF(trajectoryEN.last(), enPoint).length() > 0.5) {
        trajectoryEN.push_back(enPoint);
        if (trajectoryEN.size() > 200) {
            trajectoryEN.pop_front();
        }
    }

    update();
}

void LandingView::setVelocityENU(const Vector3& vel)
{
    velocityENU = vel;
    update();
}

void LandingView::setYawDeg(double yaw)
{
    yawDeg = yaw;
    update();
}

void LandingView::setTargetENU(const Vector3& target)
{
    targetENU = target;
    update();
}

void LandingView::setThrust(double percent)
{
    percent *= 100;
    thrustPercent = qBound(0.0, percent, 100.0);
    update();
}

void LandingView::setRCSActive(Vector3 thrust)
{
    if (thrust.x != 0 || thrust.y != 0 || thrust.z < 0)
    {
        RCSActive = true;
    }
    else
    {
        RCSActive = false;
    }
}

void LandingView::setHullIntact(SpacecraftState spacecraftState_)
{
    hullIntact =
        spacecraftState_ == SpacecraftState::Operational ||
        spacecraftState_ == SpacecraftState::Landed;

    update();
}

void LandingView::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    p.fillRect(rect(), QColor("#0E1624"));

    const int margin = 10;
    const int gap = 8;

    QRect content = rect().adjusted(margin, margin, -margin, -margin);

    const int topHeight = static_cast<int>(content.height() * 0.58);
    const int bottomHeight = content.height() - topHeight - gap;

    QRect sideRect(content.left(), content.top(), content.width(), topHeight);

    const int statusWidth = static_cast<int>(content.width() * 0.34);
    QRect statusRect(content.left(),
                     sideRect.bottom() + gap,
                     statusWidth,
                     bottomHeight);

    QRect topRect(statusRect.right() + gap,
                  sideRect.bottom() + gap,
                  content.width() - statusWidth - gap,
                  bottomHeight);

    drawSideView(p, sideRect);
    drawStatusBox(p, statusRect);
    drawTopView(p, topRect);
}

void LandingView::drawPanelFrame(QPainter& p, const QRect& r, const QString& title)
{
    p.setPen(QPen(QColor("#2F4A72"), 1));
    p.setBrush(QColor(10, 18, 30, 120));
    p.drawRoundedRect(r, 6, 6);

    QRect titleRect = QRect(r.left() + 8, r.top() + 6, r.width() - 16, 16);
    p.setPen(QColor("#4FC3F7"));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, title);
}

QPointF LandingView::mapSideView(const QRect& r, double east, double up,
                                 double maxEastAbs, double maxUp) const
{
    const QRect plot = r.adjusted(12, 26, -12, -12);

    const double xNorm = (east + maxEastAbs) / (2.0 * maxEastAbs);
    const double yNorm = 1.0 - qBound(0.0, up / maxUp, 1.0);

    return QPointF(plot.left() + xNorm * plot.width(),
                   plot.top() + yNorm * plot.height());
}

QPointF LandingView::mapTopView(const QRect& r, double east, double north,
                                double maxEastAbs, double maxNorthAbs) const
{
    const QRect plot = r.adjusted(12, 26, -12, -12);

    const double xNorm = (east + maxEastAbs) / (2.0 * maxEastAbs);
    const double yNorm = 1.0 - ((north + maxNorthAbs) / (2.0 * maxNorthAbs));

    return QPointF(plot.left() + xNorm * plot.width(),
                   plot.top() + yNorm * plot.height());
}

void LandingView::drawLanderSide(QPainter& p, const QPointF& center)
{
    QColor hullColor = hullIntact ? QColor("#CFD8DC") : QColor("#E53935");
    p.setPen(QPen(Qt::black, 1));
    p.setBrush(hullColor);

    QRectF body(center.x() - 10, center.y() - 14, 20, 20);
    p.drawRoundedRect(body, 3, 3);

    p.setPen(QPen(QColor("#90A4AE"), 2));
    p.drawLine(QPointF(body.left() + 2, body.bottom()),
               QPointF(body.left() - 8, body.bottom() + 10));
    p.drawLine(QPointF(body.right() - 2, body.bottom()),
               QPointF(body.right() + 8, body.bottom() + 10));

    p.setBrush(QColor("#4FC3F7"));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(center.x(), center.y() - 5), 3, 3);
}

void LandingView::drawLanderTop(QPainter& p, const QPointF& center, double yawDeg_)
{
    QColor hullColor = hullIntact ? QColor("#CFD8DC") : QColor("#E53935");

    p.save();
    p.translate(center);
    p.rotate(-yawDeg_);

    p.setPen(QPen(Qt::black, 1));
    p.setBrush(hullColor);
    p.drawRoundedRect(QRectF(-10, -10, 20, 20), 3, 3);

    p.setPen(QPen(QColor("#4FC3F7"), 2));
    p.drawLine(QPointF(0, 0), QPointF(0, -14));

    p.restore();
}

void LandingView::drawVector(QPainter& p, const QPointF& start, const QPointF& vec, const QColor& color)
{
    p.setPen(QPen(color, 2));
    QPointF end = start + vec;
    p.drawLine(start, end);

    QLineF line(start, end);
    if (line.length() < 1.0) {
        return;
    }

    const double arrowSize = 6.0;
    QLineF left = line;
    left.setLength(arrowSize);
    left.setAngle(line.angle() + 150);

    QLineF right = line;
    right.setLength(arrowSize);
    right.setAngle(line.angle() - 150);

    p.drawLine(end, left.p2());
    p.drawLine(end, right.p2());
}

void LandingView::drawSideView(QPainter& p, const QRect& r)
{
    drawPanelFrame(p, r, "SIDE VIEW (E-UP)");

    const QRect plot = r.adjusted(12, 26, -12, -12);
    const int groundY = plot.bottom();

    p.setPen(QPen(QColor("#24364F"), 1));
    for (int i = 1; i < 4; ++i) {
        int x = plot.left() + i * plot.width() / 4;
        int y = plot.top() + i * plot.height() / 4;
        p.drawLine(x, plot.top(), x, plot.bottom());
        p.drawLine(plot.left(), y, plot.right(), y);
    }

    p.setPen(QPen(QColor("#6D4C41"), 3));
    p.drawLine(plot.left(), groundY, plot.right(), groundY);

    const double maxEastAbs = qMax(50.0, qAbs(positionENU.x) + 10.0);
    const double maxUp = qMax(50.0, positionENU.z + 20.0);

    QPointF landerPos = mapSideView(r, positionENU.x, positionENU.z, maxEastAbs, maxUp);
    QPointF targetPos = mapSideView(r, targetENU.x, 0.0, maxEastAbs, maxUp);

    p.setPen(QPen(QColor("#66BB6A"), 2));
    p.setBrush(QColor("#66BB6A"));
    p.drawEllipse(targetPos, 4, 4);

    p.setPen(QPen(QColor("#66BB6A"), 1, Qt::DashLine));
    p.drawLine(QPointF(targetPos.x(), plot.top()), QPointF(targetPos.x(), plot.bottom()));

    drawLanderSide(p, landerPos);

    // Velocity vector in side view: East + Up
    const QPointF velVec(positionENU.x + velocityENU.x * 2.0, positionENU.z + velocityENU.z * 2.0);
    QPointF velEnd = mapSideView(r, velVec.x(), velVec.y(), maxEastAbs, maxUp);
    drawVector(p, landerPos, velEnd - landerPos, QColor("#FFD54F"));

    // Thrust vector visually upward from lander
    if (thrustPercent > 0.0) {
        const double thrustScale = 0.35 * thrustPercent;
        drawVector(p, landerPos, QPointF(0.0, -thrustScale), QColor("#4FC3F7"));
    }

    // Motion stripes under lander
    p.setPen(QPen(QColor(255, 255, 255, 35), 2));
    for (int i = 0; i < 5; ++i) {
        int y = static_cast<int>(landerPos.y()) + 24 + (i * 14) + motionOffset;
        if (y >= groundY) {
            continue;
        }
        p.drawLine(static_cast<int>(landerPos.x()) - 24, y,
                   static_cast<int>(landerPos.x()) + 24, y);
    }
}

void LandingView::drawTopView(QPainter& p, const QRect& r)
{
    drawPanelFrame(p, r, "TOP VIEW (E-N)");

    const QRect plot = r.adjusted(12, 26, -12, -12);

    p.setPen(QPen(QColor("#24364F"), 1));
    for (int i = 1; i < 4; ++i) {
        int x = plot.left() + i * plot.width() / 4;
        int y = plot.top() + i * plot.height() / 4;
        p.drawLine(x, plot.top(), x, plot.bottom());
        p.drawLine(plot.left(), y, plot.right(), y);
    }

    // axes through center
    p.setPen(QPen(QColor("#355070"), 1, Qt::DashLine));
    p.drawLine(plot.center().x(), plot.top(), plot.center().x(), plot.bottom());
    p.drawLine(plot.left(), plot.center().y(), plot.right(), plot.center().y());

    const double maxEastAbs = qMax(50.0, qMax(qAbs(positionENU.x), qAbs(targetENU.x)) + 10.0);
    const double maxNorthAbs = qMax(50.0, qMax(qAbs(positionENU.y), qAbs(targetENU.y)) + 10.0);

    QPointF landerPos = mapTopView(r, positionENU.x, positionENU.y, maxEastAbs, maxNorthAbs);
    QPointF targetPos = mapTopView(r, targetENU.x, targetENU.y, maxEastAbs, maxNorthAbs);

    // trail
    if (trajectoryEN.size() > 1) {
        p.setPen(QPen(QColor("#4FC3F7"), 1));
        for (int i = 1; i < trajectoryEN.size(); ++i) {
            QPointF a = mapTopView(r, trajectoryEN[i - 1].x(), trajectoryEN[i - 1].y(), maxEastAbs, maxNorthAbs);
            QPointF b = mapTopView(r, trajectoryEN[i].x(), trajectoryEN[i].y(), maxEastAbs, maxNorthAbs);
            p.drawLine(a, b);
        }
    }

    p.setPen(QPen(QColor("#66BB6A"), 2));
    p.setBrush(QColor("#66BB6A"));
    p.drawEllipse(targetPos, 4, 4);

    drawLanderTop(p, landerPos, yawDeg);

    // lateral velocity vector (E,N)
    QPointF velEnd = mapTopView(r,
                                positionENU.x + velocityENU.x * 2.0,
                                positionENU.y + velocityENU.y * 2.0,
                                maxEastAbs,
                                maxNorthAbs);
    drawVector(p, landerPos, velEnd - landerPos, QColor("#FFD54F"));
}

void LandingView::drawStatusBox(QPainter& p, const QRect& r)
{
    drawPanelFrame(p, r, "STATE");

    QRect textRect = r.adjusted(12, 28, -12, -12);
    p.setPen(QColor("#D6E1F0"));

    const double lateralSpeed = qSqrt(velocityENU.x * velocityENU.x + velocityENU.y * velocityENU.y);

    QStringList lines;
    lines << QString("E  : %1 m").arg(positionENU.x, 0, 'f', 1)
          << QString("N  : %1 m").arg(positionENU.y, 0, 'f', 1)
          << QString("U  : %1 m").arg(positionENU.z, 0, 'f', 1)
          << QString("VE : %1 m/s").arg(velocityENU.x, 0, 'f', 1)
          << QString("VN : %1 m/s").arg(velocityENU.y, 0, 'f', 1)
          << QString("VU : %1 m/s").arg(velocityENU.z, 0, 'f', 1)
          << QString("Vlat: %1 m/s").arg(lateralSpeed, 0, 'f', 1)
          << QString("Yaw : %1 deg").arg(yawDeg, 0, 'f', 1)
          << QString("Main Throttle [%]: %1 %").arg(thrustPercent, 0, 'f', 0)
          << QString("RCS : %1").arg(RCSActive ? "ACTIVE" : "INACTIVE")
          << QString("Frame : LOCAL ENU")
          << QString("Ref   : MCI")
          << QString("Axes  : X=E  Y=N  Z=U");

    const int lineHeight = 16;
    for (int i = 0; i < lines.size(); ++i) {
        QRect lineRect(textRect.left(), textRect.top() + i * lineHeight, textRect.width(), lineHeight);
        p.drawText(lineRect, Qt::AlignLeft | Qt::AlignVCenter, lines[i]);
    }
}
