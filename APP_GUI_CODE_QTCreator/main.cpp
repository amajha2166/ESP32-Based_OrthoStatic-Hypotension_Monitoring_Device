// Orthostatic Hypotension Monitor - Qt Widgets client (Qt 5.15+ or Qt 6)
// Connects to the ESP32 TCP server (default port 8888), shows live data,
// raises desktop alerts, lets you calibrate postures and report dizziness,
// and evaluates cuff blood-pressure readings against the standard criteria.

#include <QtWidgets>
#include <QtNetwork>
#include <cmath>
#include <limits>

static const double NaN = std::numeric_limits<double>::quiet_NaN();

static QString fmt(double v, int d = 1) {
    return std::isnan(v) ? QString() : QString::number(v, 'f', d);
}

// ---------------------------------------------------------------------
//  Simple scrolling line plot (no extra Qt modules needed)
// ---------------------------------------------------------------------
class PlotWidget : public QWidget {
public:
    PlotWidget(const QString &title, const QString &unit, double yMin, double yMax,
               const QColor &color, QWidget *parent = nullptr)
        : QWidget(parent), m_title(title), m_unit(unit), m_min(yMin), m_max(yMax), m_color(color) {
        setMinimumHeight(150);
    }
    void addPoint(double v) {
        m_data.append(v);
        if (m_data.size() > MAXPTS) m_data.removeFirst();
        update();
    }
    void setBaseline(double b) { m_base = b; update(); }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), QColor("#fafafa"));
        QRectF r(48, 24, width() - 58, height() - 34);

        double last = NaN;
        for (int i = m_data.size() - 1; i >= 0; --i)
            if (!std::isnan(m_data[i])) { last = m_data[i]; break; }
        p.setPen(QColor("#222222"));
        p.drawText(8, 16, m_title + (std::isnan(last) ? QString("   --")
                                                      : QString("   %1 %2").arg(last, 0, 'f', 0).arg(m_unit)));

        for (int g = 0; g <= 4; ++g) {
            double y = r.top() + r.height() * g / 4.0;
            p.setPen(QColor("#dddddd"));
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
            double val = m_max - (m_max - m_min) * g / 4.0;
            p.setPen(QColor("#666666"));
            p.drawText(QRectF(0, y - 8, 44, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(val, 'f', 0));
        }

        auto yOf = [&](double v) {
            v = qBound(m_min, v, m_max);
            return r.bottom() - (v - m_min) / (m_max - m_min) * r.height();
        };

        if (!std::isnan(m_base)) {
            p.setPen(QPen(QColor("#888888"), 1, Qt::DashLine));
            double y = yOf(m_base);
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
            p.drawText(QPointF(r.left() + 4, y - 3), QString("resting baseline"));
        }

        p.setPen(QPen(m_color, 2));
        double step = r.width() / (MAXPTS - 1);
        int n = m_data.size();
        QPainterPath path;
        bool down = false;
        for (int i = 0; i < n; ++i) {
            double v = m_data[i];
            if (std::isnan(v)) { down = false; continue; }
            QPointF pt(r.right() - (n - 1 - i) * step, yOf(v));
            if (!down) { path.moveTo(pt); down = true; } else path.lineTo(pt);
        }
        p.drawPath(path);
    }

private:
    static const int MAXPTS = 600;   // 5 minutes at 2 Hz
    QString m_title, m_unit;
    double m_min, m_max;
    QColor m_color;
    QVector<double> m_data;
    double m_base = NaN;
};

// ---------------------------------------------------------------------
//  Main window
// ---------------------------------------------------------------------
class MainWindow : public QMainWindow {
public:
    MainWindow() {
        setWindowTitle("Orthostatic Hypotension Monitor");
        buildUi();

        m_tray = new QSystemTrayIcon(style()->standardIcon(QStyle::SP_MessageBoxWarning), this);
        m_tray->show();

        QSettings s("OrthoMon", "OrthoMonitor");
        m_ip->setText(s.value("ip", "192.168.1.50").toString());
        m_port->setValue(s.value("port", 8888).toInt());

        connect(&m_sock, &QTcpSocket::readyRead, this, [this] { onReadyRead(); });
        connect(&m_sock, &QTcpSocket::stateChanged, this,
                [this](QAbstractSocket::SocketState st) { onState(st); });
        connect(&m_reconnect, &QTimer::timeout, this, [this] { tryConnect(); });
        m_reconnect.start(3000);
        onState(QAbstractSocket::UnconnectedState);
    }

private:
    // ---------- UI helpers ----------
    static QLabel *bigLabel(const QString &text, int pt) {
        auto *l = new QLabel(text);
        QFont f = l->font();
        f.setPointSize(pt);
        f.setBold(true);
        l->setFont(f);
        l->setAlignment(Qt::AlignCenter);
        return l;
    }
    static QGroupBox *card(const QString &title, QWidget *content) {
        auto *g = new QGroupBox(title);
        auto *l = new QVBoxLayout(g);
        l->addWidget(content);
        return g;
    }
    static QSpinBox *spin(int lo, int hi, int v) {
        auto *s = new QSpinBox;
        s->setRange(lo, hi);
        s->setValue(v);
        s->setSuffix(" mmHg");
        return s;
    }

    void buildUi() {
        auto *central = new QWidget;
        auto *root = new QVBoxLayout(central);

        // --- connection row ---
        auto *conn = new QHBoxLayout;
        m_ip = new QLineEdit;
        m_ip->setPlaceholderText("ESP32 IP address (see Serial Monitor)");
        m_port = new QSpinBox;
        m_port->setRange(1, 65535);
        m_connectBtn = new QPushButton("Connect");
        m_status = new QLabel("Disconnected");
        conn->addWidget(new QLabel("ESP32 IP:"));
        conn->addWidget(m_ip, 1);
        conn->addWidget(new QLabel("Port:"));
        conn->addWidget(m_port);
        conn->addWidget(m_connectBtn);
        conn->addWidget(m_status, 1);
        root->addLayout(conn);

        // --- alert banner ---
        m_banner = new QLabel("Not connected");
        m_banner->setAlignment(Qt::AlignCenter);
        m_banner->setMinimumHeight(60);
        QFont bf = m_banner->font();
        bf.setPointSize(16);
        bf.setBold(true);
        m_banner->setFont(bf);
        setBanner("Not connected", "#757575");
        root->addWidget(m_banner);

        // --- cards ---
        auto *cards = new QHBoxLayout;
        m_hrLabel = bigLabel("--", 30);
        m_spo2Label = bigLabel("--", 30);
        m_postureLabel = bigLabel("UNKNOWN", 26);
        m_sensorLabel = new QLabel("-");
        cards->addWidget(card("Heart rate", m_hrLabel));
        cards->addWidget(card("SpO2", m_spo2Label));
        cards->addWidget(card("Posture", m_postureLabel));
        cards->addWidget(card("Sensors", m_sensorLabel));
        root->addLayout(cards);

        // --- plots + test panel ---
        auto *mid = new QHBoxLayout;
        auto *plots = new QVBoxLayout;
        m_hrPlot = new PlotWidget("Heart rate", "bpm", 40, 160, QColor("#d32f2f"));
        m_spo2Plot = new PlotWidget("SpO2", "%", 80, 100, QColor("#1976d2"));
        plots->addWidget(m_hrPlot);
        plots->addWidget(m_spo2Plot);
        mid->addLayout(plots, 3);

        auto *right = new QVBoxLayout;
        auto *testBox = new QGroupBox("Orthostatic test (heart-rate response)");
        auto *form = new QFormLayout(testBox);
        m_stateLabel = new QLabel("-");
        m_restLabel = new QLabel("-");
        m_baseLabel = new QLabel("-");
        m_deltaLabel = new QLabel("-");
        m_maxLabel = new QLabel("-");
        m_d60Label = new QLabel("-");
        m_progress = new QProgressBar;
        m_progress->setRange(0, 100);
        m_progress->setValue(0);
        form->addRow("State:", m_stateLabel);
        form->addRow("Rest posture:", m_restLabel);
        form->addRow("Resting HR:", m_baseLabel);
        form->addRow("HR change now:", m_deltaLabel);
        form->addRow("Max HR change:", m_maxLabel);
        form->addRow("HR change @ 60 s:", m_d60Label);
        form->addRow("Progress:", m_progress);
        right->addWidget(testBox);

        auto *ctlBox = new QGroupBox("Controls");
        auto *ctl = new QGridLayout(ctlBox);
        auto *dizzyBtn = new QPushButton("I FEEL DIZZY");
        dizzyBtn->setMinimumHeight(48);
        dizzyBtn->setStyleSheet("background:#c62828;color:white;font-weight:bold;font-size:15px;");
        auto *calStand = new QPushButton("Calibrate STANDING");
        auto *calSit = new QPushButton("Calibrate SITTING");
        auto *calLie = new QPushButton("Calibrate LYING");
        auto *resetBtn = new QPushButton("Reset test");
        auto *notifyBtn = new QPushButton("Test phone notification");
        auto *exportBtn = new QPushButton("Export CSV");
        ctl->addWidget(dizzyBtn, 0, 0, 1, 2);
        ctl->addWidget(calStand, 1, 0);
        ctl->addWidget(calSit, 1, 1);
        ctl->addWidget(calLie, 2, 0);
        ctl->addWidget(resetBtn, 2, 1);
        ctl->addWidget(notifyBtn, 3, 0);
        ctl->addWidget(exportBtn, 3, 1);
        right->addWidget(ctlBox);
        right->addStretch();
        mid->addLayout(right, 2);
        root->addLayout(mid, 1);

        // --- BP panel + log ---
        auto *bottom = new QHBoxLayout;
        auto *bpBox = new QGroupBox("Blood pressure check (manual cuff readings)");
        auto *bp = new QGridLayout(bpBox);
        bp->addWidget(new QLabel("Systolic"), 0, 1);
        bp->addWidget(new QLabel("Diastolic"), 0, 2);
        const char *rows[3] = {"Resting (lying/sitting)", "Standing 1 min", "Standing 3 min"};
        int defS[3] = {120, 120, 120}, defD[3] = {80, 80, 80};
        for (int r = 0; r < 3; ++r) {
            bp->addWidget(new QLabel(rows[r]), r + 1, 0);
            m_bp[r][0] = spin(50, 260, defS[r]);
            m_bp[r][1] = spin(30, 160, defD[r]);
            bp->addWidget(m_bp[r][0], r + 1, 1);
            bp->addWidget(m_bp[r][1], r + 1, 2);
        }
        auto *evalBtn = new QPushButton("Evaluate");
        m_bpResult = new QLabel("Enter readings and press Evaluate.");
        m_bpResult->setWordWrap(true);
        bp->addWidget(evalBtn, 4, 0);
        bp->addWidget(m_bpResult, 4, 1, 1, 2);
        bottom->addWidget(bpBox, 2);

        m_log = new QPlainTextEdit;
        m_log->setReadOnly(true);
        m_log->setMaximumBlockCount(500);
        bottom->addWidget(card("Event log", m_log), 3);
        root->addLayout(bottom);

        setCentralWidget(central);

        // --- connections ---
        connect(m_connectBtn, &QPushButton::clicked, this, [this] {
            if (m_autoConnect) {
                m_autoConnect = false;
                m_sock.abort();
            } else {
                QSettings s("OrthoMon", "OrthoMonitor");
                s.setValue("ip", m_ip->text().trimmed());
                s.setValue("port", m_port->value());
                m_autoConnect = true;
                tryConnect();
            }
            onState(m_sock.state());
        });
        connect(dizzyBtn, &QPushButton::clicked, this, [this] { sendCmd("DIZZY"); });
        connect(calStand, &QPushButton::clicked, this, [this] {
            if (confirmPose("STANDING upright")) sendCmd("CAL_STAND"); });
        connect(calSit, &QPushButton::clicked, this, [this] {
            if (confirmPose("SITTING (normal sitting position)")) sendCmd("CAL_SIT"); });
        connect(calLie, &QPushButton::clicked, this, [this] {
            if (confirmPose("LYING flat")) sendCmd("CAL_LIE"); });
        connect(resetBtn, &QPushButton::clicked, this, [this] { sendCmd("RESET"); });
        connect(notifyBtn, &QPushButton::clicked, this, [this] { sendCmd("TEST_NOTIFY"); });
        connect(exportBtn, &QPushButton::clicked, this, [this] { exportCsv(); });
        connect(evalBtn, &QPushButton::clicked, this, [this] { evaluateBp(); });
    }

    bool confirmPose(const QString &pose) {
        return QMessageBox::question(this, "Calibrate posture",
                   QString("Get into the %1 position with the sensor worn, hold still, then press Yes.").arg(pose))
               == QMessageBox::Yes;
    }

    void setBanner(const QString &text, const QString &color) {
        m_banner->setText(text);
        m_banner->setStyleSheet(QString("background:%1;color:white;border-radius:8px;padding:8px;").arg(color));
    }

    void logLine(const QString &s) {
        m_log->appendPlainText(QTime::currentTime().toString("HH:mm:ss") + "  " + s);
    }

    void sendCmd(const QString &c) {
        if (m_sock.state() == QAbstractSocket::ConnectedState) {
            m_sock.write((c + "\n").toUtf8());
        } else {
            logLine("Not connected - command not sent: " + c);
        }
    }

    // ---------- networking ----------
    void tryConnect() {
        if (!m_autoConnect || m_sock.state() != QAbstractSocket::UnconnectedState) return;
        m_sock.connectToHost(m_ip->text().trimmed(), quint16(m_port->value()));
    }

    void onState(QAbstractSocket::SocketState st) {
        switch (st) {
        case QAbstractSocket::ConnectedState:
            m_status->setText("Connected");
            m_status->setStyleSheet("color:#2e7d32;font-weight:bold;");
            m_connectBtn->setText("Disconnect");
            logLine("Connected to ESP32");
            break;
        case QAbstractSocket::UnconnectedState:
            m_status->setText(m_autoConnect ? "Disconnected - retrying..." : "Disconnected");
            m_status->setStyleSheet("color:#c62828;font-weight:bold;");
            m_connectBtn->setText(m_autoConnect ? "Stop" : "Connect");
            setBanner("Not connected", "#757575");
            break;
        default:
            m_status->setText("Connecting...");
            m_status->setStyleSheet("color:#ef6c00;font-weight:bold;");
            m_connectBtn->setText("Stop");
            break;
        }
    }

    void onReadyRead() {
        while (m_sock.canReadLine()) {
            QByteArray line = m_sock.readLine().trimmed();
            if (line.isEmpty()) continue;
            QJsonParseError err;
            QJsonDocument doc = QJsonDocument::fromJson(line, &err);
            if (err.error != QJsonParseError::NoError || !doc.isObject()) continue;
            QJsonObject o = doc.object();
            QString type = o.value(QString("type")).toString();
            if (type == "vitals") handleVitals(o);
            else if (type == "event") logLine(o.value(QString("msg")).toString());
        }
    }

    // ---------- data handling ----------
    void handleVitals(const QJsonObject &o) {
        auto V = [&](const char *k) { return o.value(QString::fromLatin1(k)); };
        auto num = [&](const char *k) { QJsonValue v = V(k); return v.isDouble() ? v.toDouble() : NaN; };

        double hr = num("hr_bpm"), spo2 = num("spo2_pct");
        double baseline = num("baseline_hr"), delta = num("hr_delta"), maxd = num("max_delta"), d60 = num("delta_60s");
        double rssi = num("rssi");
        QString posture = V("posture").toString("UNKNOWN");
        QString state = V("ortho_state").toString("WAIT_REST");
        QString alert = V("alert").toString("NONE");
        QString rest = V("rest_posture").toString("-");
        int pe = V("phase_elapsed_s").toInt(), pt = V("phase_total_s").toInt();
        bool finger = V("finger").toBool(), mpuOk = V("mpu_ok").toBool(), maxOk = V("max_ok").toBool();
        bool calS = V("cal_stand").toBool(), calT = V("cal_sit").toBool(), calL = V("cal_lie").toBool();

        m_hrLabel->setText(std::isnan(hr) ? "--" : QString("%1 bpm").arg(hr, 0, 'f', 0));
        m_spo2Label->setText(std::isnan(spo2) ? "--" : QString("%1 %").arg(spo2, 0, 'f', 0));

        QString pc = "#757575";
        if (posture == "STANDING") pc = "#2e7d32";
        else if (posture == "SITTING") pc = "#ef6c00";
        else if (posture == "LYING") pc = "#1565c0";
        m_postureLabel->setText(posture);
        m_postureLabel->setStyleSheet(QString("color:%1;").arg(pc));

        m_sensorLabel->setText(QString("Pulse sensor: %1\nMotion sensor: %2\nFinger: %3\nWiFi signal: %4\n"
                                       "Calibrated: stand %5, sit %6, lie %7")
                                   .arg(maxOk ? "OK" : "FAIL").arg(mpuOk ? "OK" : "FAIL")
                                   .arg(finger ? "detected" : "not detected")
                                   .arg(std::isnan(rssi) ? QString("-") : QString("%1 dBm").arg(rssi, 0, 'f', 0))
                                   .arg(calS ? "yes" : "NO").arg(calT ? "yes" : "NO").arg(calL ? "yes" : "NO"));

        m_stateLabel->setText(state);
        m_restLabel->setText(rest);
        m_baseLabel->setText(std::isnan(baseline) ? "-" : QString("%1 bpm").arg(baseline, 0, 'f', 0));
        m_deltaLabel->setText(std::isnan(delta) ? "-" : QString("%1%2 bpm").arg(delta >= 0 ? "+" : "").arg(delta, 0, 'f', 0));
        m_maxLabel->setText(std::isnan(maxd) ? "-" : QString("%1%2 bpm").arg(maxd >= 0 ? "+" : "").arg(maxd, 0, 'f', 0));
        m_d60Label->setText(std::isnan(d60) ? "-" : QString("%1%2 bpm").arg(d60 >= 0 ? "+" : "").arg(d60, 0, 'f', 0));
        m_progress->setValue(pt > 0 ? qBound(0, pe * 100 / pt, 100) : 0);
        m_progress->setFormat(pt > 0 ? QString("%1 / %2 s").arg(pe).arg(pt) : QString("-"));

        m_hrPlot->setBaseline(baseline);
        m_hrPlot->addPoint(hr);
        m_spo2Plot->addPoint(spo2);

        updateBanner(alert, state, pe, pt);

        if (m_rec.size() < 200000) m_rec.append({QDateTime::currentDateTime(), hr, spo2, posture, state, delta, alert});
    }

    void updateBanner(const QString &alert, const QString &state, int pe, int pt) {
        QString text, color;
        if (alert == "ORTHO_HR_DIZZY") {
            text = "ORTHOSTATIC ALERT: large HR rise AND dizziness reported";
            color = "#b71c1c";
        } else if (alert == "ORTHO_HR") {
            text = "POSSIBLE ORTHOSTATIC HYPOTENSION: HR rise >= 30 bpm after standing";
            color = "#c62828";
        } else if (alert == "DIZZY") {
            text = "DIZZINESS REPORTED";
            color = "#ef6c00";
        } else if (state == "WAIT_REST") {
            text = "Sit or lie down and stay still to begin";
            color = "#757575";
        } else if (state == "BASELINE") {
            text = QString("Collecting resting baseline... %1 / %2 s").arg(pe).arg(pt);
            color = "#f9a825";
        } else if (state == "READY") {
            text = "Baseline ready - STAND UP now";
            color = "#1565c0";
        } else if (state == "TESTING") {
            text = QString("Standing test running... %1 / %2 s").arg(pe).arg(pt);
            color = "#1565c0";
        } else if (state == "DONE") {
            text = "Test complete - HR criterion not met";
            color = "#2e7d32";
        } else {
            text = state;
            color = "#757575";
        }
        setBanner(text, color);

        if (alert != m_lastAlert) {
            if (alert != "NONE") {
                QApplication::beep();
                QApplication::alert(this, 0);
                m_tray->showMessage("Orthostatic Monitor", text, QSystemTrayIcon::Critical, 15000);
                logLine("ALERT: " + text);
            }
            m_lastAlert = alert;
        }
    }

    void evaluateBp() {
        int rs = m_bp[0][0]->value(), rd = m_bp[0][1]->value();
        int s1 = m_bp[1][0]->value(), d1 = m_bp[1][1]->value();
        int s3 = m_bp[2][0]->value(), d3 = m_bp[2][1]->value();
        int dS1 = rs - s1, dD1 = rd - d1, dS3 = rs - s3, dD3 = rd - d3;
        bool met = dS1 >= 20 || dD1 >= 10 || dS3 >= 20 || dD3 >= 10;
        QString t = QString("Drop at 1 min: SBP %1 / DBP %2 mmHg.  Drop at 3 min: SBP %3 / DBP %4 mmHg.\n")
                        .arg(dS1).arg(dD1).arg(dS3).arg(dD3);
        if (met) {
            t += "Criteria MET (SBP drop >= 20 or DBP drop >= 10 within 3 min): consistent with orthostatic hypotension. "
                 "Please consult a clinician.";
            m_bpResult->setStyleSheet("color:#c62828;font-weight:bold;");
        } else {
            t += "Criteria not met for these readings.";
            m_bpResult->setStyleSheet("color:#2e7d32;font-weight:bold;");
        }
        m_bpResult->setText(t);
        logLine("BP evaluation: " + QString(met ? "criteria MET" : "criteria not met"));
    }

    void exportCsv() {
        QString fn = QFileDialog::getSaveFileName(this, "Export CSV", "orthomon.csv", "CSV (*.csv)");
        if (fn.isEmpty()) return;
        QFile f(fn);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "Export", "Cannot write file.");
            return;
        }
        QTextStream ts(&f);
        ts << "time,hr_bpm,spo2_pct,posture,ortho_state,hr_delta,alert\n";
        for (const Rec &r : m_rec) {
            ts << r.time.toString(Qt::ISODate) << ',' << fmt(r.hr) << ',' << fmt(r.spo2, 0) << ','
               << r.posture << ',' << r.state << ',' << fmt(r.delta) << ',' << r.alert << '\n';
        }
        logLine(QString("Exported %1 rows").arg(m_rec.size()));
    }

    // ---------- members ----------
    struct Rec { QDateTime time; double hr, spo2; QString posture, state; double delta; QString alert; };
    QVector<Rec> m_rec;

    QTcpSocket m_sock;
    QTimer m_reconnect;
    bool m_autoConnect = false;
    QString m_lastAlert = "NONE";
    QSystemTrayIcon *m_tray = nullptr;

    QLineEdit *m_ip = nullptr;
    QSpinBox *m_port = nullptr;
    QPushButton *m_connectBtn = nullptr;
    QLabel *m_status = nullptr, *m_banner = nullptr;
    QLabel *m_hrLabel = nullptr, *m_spo2Label = nullptr, *m_postureLabel = nullptr, *m_sensorLabel = nullptr;
    QLabel *m_stateLabel = nullptr, *m_restLabel = nullptr, *m_baseLabel = nullptr;
    QLabel *m_deltaLabel = nullptr, *m_maxLabel = nullptr, *m_d60Label = nullptr, *m_bpResult = nullptr;
    QProgressBar *m_progress = nullptr;
    PlotWidget *m_hrPlot = nullptr, *m_spo2Plot = nullptr;
    QSpinBox *m_bp[3][2] = {};
    QPlainTextEdit *m_log = nullptr;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w;
    w.resize(1200, 860);
    w.show();
    return app.exec();
}
