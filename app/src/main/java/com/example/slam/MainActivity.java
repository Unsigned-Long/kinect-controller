package com.example.slam;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.google.android.material.tabs.TabLayout;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.text.DecimalFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.LinkedList;
import java.util.List;

public class MainActivity extends AppCompatActivity {
    public static final int DEVICE_CONNECTING = 1;
    public static final int DEVICE_CONNECTED = 2;
    public static final int SEND_MSG_SUCCESS = 3;
    public static final int SEND_MSG_ERROR = 4;
    public static final int GET_COLOR_IMG_SUCCESS = 5;
    public static final int GET_MSG_SUCCESS = 6;
    public static final int GET_DEPTH_IMG_SUCCESS = 7;

    // this is the port
    public static final int DEVICE_MSG_PORT = 3890;
    public static final int DEVICE_COLOR_IMG_PORT = 9988;
    public static final int DEVICE_DEPTH_IMG_PORT = 1314;

    private static final String _targetWiFiSSID = "slam-device";
    private static final String _targetHostName = "slam-device";

    //    private static final String _targetWiFiSSID = "Juster";
    //    private static final String _targetHostName = "whu-ubuntu";

    // message flags
    private static final String COLOR_DEPTH_TIME = "0";
    private static final String INFO_MESSAGE_TEXTVIEW = "1";
    private static final String INFO_MESSAGE_TOAST = "2";
    private static final String IMU_MESSAGE = "3";

    // max length for line chart drawing
    private final Integer _MAX_LIST_ITEMS = 20;

    // wifi
    private WifiManager _wifiManager;
    private WifiInfo _wifiInfo;

    //  "0": wifi not turned on
    //  "1": wifi turned on and connect to target wifi
    // "-1": wifi turned on but not connect to target wifi
    private int _targetWiFiConnected;
    private WifiStateBroadcastReceive _wifiStateReceiver;

    // textview [wifi info]
    private TextView _tv_deviceConnectState;
    private TextView _tv_timeDisplay_wifi, _tv_timeDisplay_home, _tv_timeDisplay_help;
    private TextView _tv_ssid;
    private TextView _tv_rssi;
    private TextView _tv_speed;
    private TextView _tv_frequency;
    private TextView _tv_netId;
    private TextView _tv_deviceMsg;
    private TextView _tv_rgbPath, _tv_depthPath, _tv_timeStamp;
    private TextView _tv_supState;

    // imu
    private TextView _tv_ax;
    private TextView _tv_ay;
    private TextView _tv_az;
    private TextView _tv_gx;
    private TextView _tv_gy;
    private TextView _tv_gz;
    private LinkedList<DataItem> _acceleration;
    private LineChart _acceLineChart;
    private LinkedList<DataItem> _gyroscope;
    private LineChart _gyroLineChart;

    // image construct
    private byte[] _colorImageBuffer;
    private int _curColorImageBufferIdx;

    private byte[] _depthImageBuffer;
    private int _curdepthImageBufferIdx;

    // image
    private ImageView _iv_letter_s, _iv_letter_l, _iv_letter_a, _iv_letter_m;

    // double buffer
    private SurfaceView _sv_colorImage;
    private SurfaceView _sv_depthImage;

    // handler
    private Handler _timeDisplayHandler;
    private Handler _msgHandler;

    // frame
    private FrameLayout _frameSlam, _frameWifi, _frameHelp;

    // tcp [msg, img]
    private ConnectThread _connectionForMsg;
    private ListenerThread _listenerMsgThread;
    private ConnectThread _connectionForColorImg;
    private ListenerThread _listenerColorImgThread;
    private ConnectThread _connectionForDepthImg;
    private ListenerThread _listenerDepthImgThread;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // init the variables
        this.initVariables();
        // register the wifi manager
        this.getWifiAdmin();
        // register the wifi receive [wifi state, wifi connection state]
        this.registerWifiReceiver();

    }

    public String intIP2StringIP(int ip) {
        return (ip & 0xFF) + "." +
                ((ip >> 8) & 0xFF) + "." +
                ((ip >> 16) & 0xFF) + "." +
                (ip >> 24 & 0xFF);
    }

    public void handleReceivedMsg(String msgString) {
        Log.d("---", msgString);

        String[] strItems = msgString.split("#");
        for (String strItem : strItems) {
            if (strItem.isEmpty()) {
                continue;
            }

            if (strItem.startsWith(INFO_MESSAGE_TEXTVIEW)) {
                _tv_deviceMsg.setText(strItem.substring(1));
            } else if (strItem.startsWith(COLOR_DEPTH_TIME)) {
                String[] strAry = strItem.substring(1).split(":");
                if (strAry.length != 3) {
                    continue;
                }
                _tv_rgbPath.setText(strAry[0]);
                _tv_depthPath.setText(strAry[1]);
                _tv_timeStamp.setText(strAry[2]);

            } else if (strItem.startsWith(INFO_MESSAGE_TOAST)) {
                _tv_deviceMsg.setText(strItem.substring(1));
                Toast.makeText(MainActivity.this, strItem.substring(1), Toast.LENGTH_SHORT).show();
            } else if (strItem.startsWith(IMU_MESSAGE)) {
                DecimalFormat format = new DecimalFormat("+00.0000;-00.0000");

                String[] strAry = strItem.substring(1).split(" ");
                if (strAry.length != 6) {
                    continue;
                }

                float ax = Float.parseFloat(strAry[0]);
                float ay = Float.parseFloat(strAry[1]);
                float az = Float.parseFloat(strAry[2]);
                this._acceleration.add(new DataItem(new float[]{ax, ay, az}));
                if (this._acceleration.size() > this._MAX_LIST_ITEMS) {
                    this._acceleration.removeFirst();
                }

                // set content for the text views
                this._tv_ax.setText(format.format(ax));
                this._tv_ay.setText(format.format(ay));
                this._tv_az.setText(format.format(az));

                this.drawLineChart(this._acceLineChart, this._acceleration, "A(x)", "A(y)", "A(z)");

                float gx = Float.parseFloat(strAry[3]);
                float gy = Float.parseFloat(strAry[4]);
                float gz = Float.parseFloat(strAry[5]);

                this._gyroscope.add(new DataItem(new float[]{gx, gy, gz}));
                if (this._gyroscope.size() > this._MAX_LIST_ITEMS) {
                    this._gyroscope.removeFirst();
                }

                // set content for the text views
                this._tv_gx.setText(format.format(gx));
                this._tv_gy.setText(format.format(gy));
                this._tv_gz.setText(format.format(gz));

                // draw
                this.drawLineChart(this._gyroLineChart, this._gyroscope, "G(x)", "G(y)", "G(z)");
            }
        }
    }

    @SuppressLint("ResourceAsColor")
    public void handleReceivedImg(byte[] buffer, int IMG_TYPE) {
        byte[] header = {buffer[0]};

        // TODO: 4/16/22 check the image left size
        if (new String(header).equals("E")) {
            // end
            byte[] length = {buffer[1], buffer[2], buffer[3], buffer[4]};
            int leftSize = Integer.valueOf(new String(length));

            if (IMG_TYPE == GET_COLOR_IMG_SUCCESS) {
                Log.d("---", "end color image, left size: [" + leftSize + "]");

                System.arraycopy(buffer, 5, _colorImageBuffer, _curColorImageBufferIdx, leftSize);
                _curColorImageBufferIdx += leftSize;

                Bitmap colorBitmap = BitmapFactory.decodeByteArray(_colorImageBuffer, 0, _curColorImageBufferIdx);
                if (colorBitmap == null) {
                    return;
                }
                Log.d("---", "handleReceivedImg: drawing a new image");
                // TODO: 4/15/22 show image
                Canvas canvas = _sv_colorImage.getHolder().lockCanvas();

                // range [source image]
                Rect srcRect = new Rect(0, 0, colorBitmap.getWidth(), colorBitmap.getHeight());
                // range [surface image]
                Rect dstRect = new Rect(0, 0, _sv_colorImage.getWidth(), _sv_colorImage.getHeight());
                canvas.drawBitmap(colorBitmap, srcRect, dstRect, null);

                _sv_colorImage.getHolder().unlockCanvasAndPost(canvas);

                _curColorImageBufferIdx = 0;
            } else if (IMG_TYPE == GET_DEPTH_IMG_SUCCESS) {
                Log.d("---", "end depth image, left size: [" + leftSize + "]");

                System.arraycopy(buffer, 5, _depthImageBuffer, _curdepthImageBufferIdx, leftSize);
                _curdepthImageBufferIdx += leftSize;

                Bitmap depthBitmap = BitmapFactory.decodeByteArray(_depthImageBuffer, 0, _curdepthImageBufferIdx);
                if (depthBitmap == null) {
                    return;
                }
                Log.d("---", "handleReceivedImg: drawing a new image");
                // TODO: 4/15/22 show image
                Canvas canvas = _sv_depthImage.getHolder().lockCanvas();
                canvas.drawColor(R.color.white);

                // range [source image]
                Rect srcRect = new Rect(0, 0, depthBitmap.getWidth(), depthBitmap.getHeight());
                // range [surface image]
                Rect dstRect = new Rect(0, 0, _sv_depthImage.getWidth(), _sv_depthImage.getHeight());
                canvas.drawBitmap(depthBitmap, srcRect, dstRect, null);

                _sv_depthImage.getHolder().unlockCanvasAndPost(canvas);

                _curdepthImageBufferIdx = 0;
            }

        } else if (new String(header).equals("M")) {
            // new image
            if (IMG_TYPE == GET_COLOR_IMG_SUCCESS) {
                System.arraycopy(buffer, 5, _colorImageBuffer, _curColorImageBufferIdx, 2043);
                _curColorImageBufferIdx += 2043;
                Log.d("---", "receiving an middle piece color image... size [2043]");
            } else {
                System.arraycopy(buffer, 5, _depthImageBuffer, _curdepthImageBufferIdx, 2043);
                _curdepthImageBufferIdx += 2043;
                Log.d("---", "receiving an middle piece depth image... size [2043]");
            }
        } else if (new String(header).equals("S")) {
            if (IMG_TYPE == GET_COLOR_IMG_SUCCESS) {
                _curColorImageBufferIdx = 0;
                System.arraycopy(buffer, 5, _colorImageBuffer, _curColorImageBufferIdx, 2043);
                _curColorImageBufferIdx += 2043;
                Log.d("---", "receiving an new color image... size [2043]");
            } else if (IMG_TYPE == GET_DEPTH_IMG_SUCCESS) {
                _curdepthImageBufferIdx = 0;
                System.arraycopy(buffer, 5, _depthImageBuffer, _curdepthImageBufferIdx, 2043);
                _curdepthImageBufferIdx += 2043;
                Log.d("---", "receiving an new depth image... size [2043]");
            }
        }
    }

    public <T extends DataItem>
    void drawLineChart(LineChart chart, LinkedList<T> data, String xName, String yName, String zName) {
        List<Entry> lsX = new LinkedList<>();
        List<Entry> lsY = new LinkedList<>();
        List<Entry> lsZ = new LinkedList<>();

        int count = 0;
        float maxAbs = 0.0f;

        for (T elem : data) {

            lsX.add(new Entry(count, elem._values[0]));
            lsY.add(new Entry(count, elem._values[1]));
            lsZ.add(new Entry(count, elem._values[2]));

            if (Math.abs(elem._values[0]) > maxAbs) {
                maxAbs = Math.abs(elem._values[0]);
            }
            if (Math.abs(elem._values[1]) > maxAbs) {
                maxAbs = Math.abs(elem._values[1]);
            }
            if (Math.abs(elem._values[2]) > maxAbs) {
                maxAbs = Math.abs(elem._values[2]);
            }

            ++count;
        }

        maxAbs *= 1.2f;


        LineDataSet dsX = new LineDataSet(lsX, xName);
        dsX.setLineWidth(1);
        dsX.setColor(Color.parseColor("#A62121"));
        dsX.setDrawValues(false);
        dsX.setCircleColor(Color.parseColor("#A62121"));

        LineDataSet dsY = new LineDataSet(lsY, yName);
        dsY.setLineWidth(1);
        dsY.setColor(Color.parseColor("#8BC34A"));
        dsY.setDrawValues(false);
        dsY.setCircleColor(Color.parseColor("#8BC34A"));

        LineDataSet dsZ = new LineDataSet(lsZ, zName);
        dsZ.setLineWidth(1);
        dsZ.setColor(Color.parseColor("#FF9800"));
        dsZ.setDrawValues(false);
        dsZ.setCircleColor(Color.parseColor("#FF9800"));

        XAxis xAxis = chart.getXAxis();
        xAxis.setAxisMinimum(0);
        xAxis.setAxisMaximum(this._MAX_LIST_ITEMS);
        xAxis.enableGridDashedLine(5, 4, 0);
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        xAxis.setDrawAxisLine(false);

        YAxis leftAxis = chart.getAxisLeft();
        leftAxis.setAxisMinimum(-maxAbs);
        leftAxis.setAxisMaximum(maxAbs);
        leftAxis.enableGridDashedLine(5, 4, 0);
        leftAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        leftAxis.setLabelCount(7);
        leftAxis.setDrawAxisLine(false);

        YAxis rightAxis = chart.getAxisRight();
        rightAxis.setAxisMinimum(-maxAbs);
        rightAxis.setAxisMaximum(maxAbs);
        rightAxis.enableGridDashedLine(5, 4, 0);
        rightAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        rightAxis.setLabelCount(7);
        rightAxis.setDrawAxisLine(false);


        chart.animateXY(0, 0);
        chart.getDescription().setTextSize(12);
        chart.getDescription().setTextColor(Color.parseColor("#DC4949"));
        chart.getDescription().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        chart.getLegend().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        chart.setDrawMarkers(true);
        LineData ld = new LineData();
        ld.addDataSet(dsX);
        ld.addDataSet(dsY);
        ld.addDataSet(dsZ);
        chart.setData(ld);
    }

    @SuppressLint("SetTextI18n")
    public void initVariables() {
        // bytes
        _colorImageBuffer = new byte[50000];
        _curColorImageBufferIdx = 0;

        _depthImageBuffer = new byte[50000];
        _curdepthImageBufferIdx = 0;

        // acce, gyro
        this._tv_ax = findViewById(R.id.acce_x);
        this._tv_ay = findViewById(R.id.acce_y);
        this._tv_az = findViewById(R.id.acce_z);

        this._tv_gx = findViewById(R.id.gyro_x);
        this._tv_gy = findViewById(R.id.gyro_y);
        this._tv_gz = findViewById(R.id.gyro_z);

        this._acceLineChart = findViewById(R.id.linechart_acce);
        this._acceLineChart.setNoDataText("NO DATA FOR ACCELERATION");
        this._acceLineChart.setNoDataTextColor(Color.RED);
        this._acceLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._acceLineChart.getDescription().setText("Unit(M/S^2)");

        this._gyroLineChart = findViewById(R.id.linechart_gyro);
        this._gyroLineChart.setNoDataText("NO DATA FOR GYROSCOPE");
        this._gyroLineChart.setNoDataTextColor(Color.RED);
        this._gyroLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._gyroLineChart.getDescription().setText("Unit(RAD/S)");

        this._acceleration = new LinkedList<>();
        this._gyroscope = new LinkedList<>();

        // frame
        _frameSlam = findViewById(R.id.frame_slam);
        _frameWifi = findViewById(R.id.frame_wifi);
        _frameHelp = findViewById(R.id.frame_help);
        TabLayout tabLayout = findViewById(R.id.nav_view);
        tabLayout.addOnTabSelectedListener(new TabLayout.OnTabSelectedListener() {
            @Override
            public void onTabSelected(TabLayout.Tab tab) {
                switch (tab.getPosition()) {
                    case 0:
                        _frameSlam.setVisibility(View.VISIBLE);
                        _frameWifi.setVisibility(View.GONE);
                        _frameHelp.setVisibility(View.GONE);
                        break;
                    case 1:
                        _frameSlam.setVisibility(View.GONE);
                        _frameWifi.setVisibility(View.VISIBLE);
                        _frameHelp.setVisibility(View.GONE);
                        break;
                    case 2:
                        _frameSlam.setVisibility(View.GONE);
                        _frameWifi.setVisibility(View.GONE);
                        _frameHelp.setVisibility(View.VISIBLE);
                        break;
                }
            }

            @Override
            public void onTabUnselected(TabLayout.Tab tab) {

            }

            @Override
            public void onTabReselected(TabLayout.Tab tab) {

            }
        });

        this._targetWiFiConnected = 0;
        this._wifiStateReceiver = new WifiStateBroadcastReceive();

        // textview
        _tv_ssid = findViewById(R.id.textview_ssid);
        _tv_rssi = findViewById(R.id.textview_rssi);
        _tv_speed = findViewById(R.id.textview_speed);
        _tv_frequency = findViewById(R.id.textview_frequency);
        _tv_netId = findViewById(R.id.textview_netId);
        _tv_deviceMsg = findViewById(R.id.textview_deviceMsg);
        _tv_rgbPath = findViewById(R.id.textview_rgb);
        _tv_depthPath = findViewById(R.id.textview_depth);
        _tv_timeStamp = findViewById(R.id.textview_imgTime);
        _tv_supState = findViewById(R.id.textview_supstate);

        // images
        _iv_letter_s = findViewById(R.id.img_s);
        _iv_letter_l = findViewById(R.id.img_l);
        _iv_letter_a = findViewById(R.id.img_a);
        _iv_letter_m = findViewById(R.id.img_m);

        _sv_colorImage = findViewById(R.id.color_image);
        _sv_colorImage.getHolder().addCallback(new SurfaceHolder.Callback() {

            @Override
            public void surfaceCreated(SurfaceHolder surfaceHolder) {
            }

            @Override
            public void surfaceChanged(@NonNull SurfaceHolder surfaceHolder, int format, int width, int height) {
                Log.d("---", "surfaceChanged");
                Paint paint = new Paint();
                paint.setAntiAlias(true);
                paint.setStyle(Paint.Style.STROKE);

                Canvas canvas = surfaceHolder.lockCanvas();

                Bitmap bitmap = BitmapFactory.decodeResource(getResources(), R.drawable.sv_init_rgb);
                int bitWidth = bitmap.getWidth();
                int bitHeight = bitmap.getHeight();
                Rect srcRect = new Rect(0, 0, bitWidth, bitHeight);
                Rect dstRect = new Rect(0, 0, width, height);

                canvas.drawBitmap(bitmap, srcRect, dstRect, paint);

                surfaceHolder.unlockCanvasAndPost(canvas);
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder surfaceHolder) {

            }

        });

        _sv_depthImage = findViewById(R.id.depth_image);
        _sv_depthImage.getHolder().addCallback(new SurfaceHolder.Callback() {

            @Override
            public void surfaceCreated(SurfaceHolder surfaceHolder) {
            }

            @Override
            public void surfaceChanged(@NonNull SurfaceHolder surfaceHolder, int format, int width, int height) {
                Log.d("---", "surfaceChanged");
                Paint paint = new Paint();
                paint.setAntiAlias(true);
                paint.setStyle(Paint.Style.STROKE);

                Canvas canvas = surfaceHolder.lockCanvas();

                Bitmap bitmap = BitmapFactory.decodeResource(getResources(), R.drawable.sv_init_depth);
                int bitWidth = bitmap.getWidth();
                int bitHeight = bitmap.getHeight();
                Rect srcRect = new Rect(0, 0, bitWidth, bitHeight);
                Rect dstRect = new Rect(0, 0, width, height);

                canvas.drawBitmap(bitmap, srcRect, dstRect, paint);

                surfaceHolder.unlockCanvasAndPost(canvas);
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder surfaceHolder) {

            }

        });

        // tcp
        this._tv_deviceConnectState = findViewById(R.id.text_state);
        // message handler
        _msgHandler = new Handler(Looper.getMainLooper()) {
            @SuppressLint("SetTextI18n")
            @Override
            public void handleMessage(Message msg) {
                switch (msg.what) {
                    case DEVICE_CONNECTING:
                        // when a new connection coming

                        // create a new message connection
                        _connectionForMsg = new ConnectThread(_listenerMsgThread.getSocket(), _msgHandler, MainActivity.GET_MSG_SUCCESS);
                        _connectionForMsg.start();

                        // create a new image connection
                        _connectionForColorImg = new ConnectThread(_listenerColorImgThread.getSocket(), _msgHandler, MainActivity.GET_COLOR_IMG_SUCCESS);
                        _connectionForColorImg.start();

                        // create a new image connection
                        _connectionForDepthImg = new ConnectThread(_listenerDepthImgThread.getSocket(), _msgHandler, MainActivity.GET_DEPTH_IMG_SUCCESS);
                        _connectionForDepthImg.start();
                        break;

                    case DEVICE_CONNECTED:
                        // show message
                        _tv_deviceConnectState.setText("Device Connected Successfully");
                        break;

                    case SEND_MSG_SUCCESS:
                        // show success message
                        _tv_deviceConnectState.setText("Message Sent Successfully [" + msg.getData().getString("MSG") + "]");
                        break;

                    case SEND_MSG_ERROR:
                        // show error message
                        _tv_deviceConnectState.setText("Failed To Send Message [" + msg.getData().getString("MSG") + "]");
                        break;

                    case GET_MSG_SUCCESS:
                        // show received message
                        String message = msg.getData().getString("MSG");
//                        Log.d("---", "I get a message, string length: " + message.length());
                        handleReceivedMsg(message);
                        break;

                    case GET_COLOR_IMG_SUCCESS:
                        byte[] bys_color = msg.getData().getByteArray("MSG");
                        handleReceivedImg(bys_color, GET_COLOR_IMG_SUCCESS);
                        break;
                    case GET_DEPTH_IMG_SUCCESS:
                        byte[] bys_depth = msg.getData().getByteArray("MSG");
                        handleReceivedImg(bys_depth, GET_DEPTH_IMG_SUCCESS);
                        break;

                    default:
                }
            }
        };

        // button click event
        Button _btnConnect = findViewById(R.id.button_connect);
        _btnConnect.setOnClickListener(view -> {

            if (_targetWiFiConnected == 0) {
                Toast.makeText(MainActivity.this, "please turn on WiFi", Toast.LENGTH_SHORT).show();
            } else if (_targetWiFiConnected == -1) {
                Toast.makeText(MainActivity.this, "please connect to " + _targetWiFiSSID, Toast.LENGTH_SHORT).show();
            } else {
                // construct the socket
                new Thread(() -> {
                    try {
                        // construct a new socket
                        Socket msg_socket = new Socket(_targetHostName, DEVICE_MSG_PORT);
                        Socket color_img_socket = new Socket(_targetHostName, DEVICE_COLOR_IMG_PORT);
                        Socket depth_img_socket = new Socket(_targetHostName, DEVICE_DEPTH_IMG_PORT);

                        // construct a new connect thread
                        _connectionForMsg = new ConnectThread(msg_socket, _msgHandler, MainActivity.GET_MSG_SUCCESS);
                        // start connect
                        _connectionForMsg.start();

                        // construct a new connect thread
                        _connectionForColorImg = new ConnectThread(color_img_socket, _msgHandler, MainActivity.GET_COLOR_IMG_SUCCESS);
                        // start connect
                        _connectionForColorImg.start();

                        // construct a new connect thread
                        _connectionForDepthImg = new ConnectThread(depth_img_socket, _msgHandler, MainActivity.GET_DEPTH_IMG_SUCCESS);
                        // start connect
                        _connectionForDepthImg.start();

                    } catch (IOException e) {
                        e.printStackTrace();
                        runOnUiThread(() -> _tv_deviceConnectState.setText("Connect failed, Server Not Running"));
                    }
                }).start();
            }
        });

        Button _btnStart = findViewById(R.id.button_start);
        _btnStart.setOnClickListener(view -> {
            if (_connectionForMsg != null) {
                new Thread(() -> _connectionForMsg.sendData("start")).start();

                Animation animation = AnimationUtils.loadAnimation(MainActivity.this, R.anim.running);

                _iv_letter_s.startAnimation(animation);
                _iv_letter_l.startAnimation(animation);
                _iv_letter_a.startAnimation(animation);
                _iv_letter_m.startAnimation(animation);

            } else {
                Toast.makeText(MainActivity.this, "device isn't connected", Toast.LENGTH_SHORT).show();
            }
        });

        Button _btnStop = findViewById(R.id.button_stop);
        _btnStop.setOnClickListener(view -> {
            if (_connectionForMsg != null) {
                new AlertDialog.Builder(this, R.style.dialogStyle)
                        .setTitle("Information")
                        .setMessage("Are you sure to stop collection?")
                        .setIcon(R.drawable.exc)
                        .setPositiveButton("Yes", (dialog, which) -> {
                            new Thread(() -> _connectionForMsg.sendData("stop")).start();
                            _iv_letter_s.clearAnimation();
                            _iv_letter_l.clearAnimation();
                            _iv_letter_a.clearAnimation();
                            _iv_letter_m.clearAnimation();
                        })
                        .setNegativeButton("Cancel", (dialog, which) -> {
                        })
                        .create()
                        .show();

            } else {
                Toast.makeText(MainActivity.this, "device isn't connected", Toast.LENGTH_SHORT).show();
            }
        });

        // listener [msg, img]
        _listenerMsgThread = new ListenerThread(DEVICE_MSG_PORT, _msgHandler);
        _listenerMsgThread.start();
        _listenerColorImgThread = new ListenerThread(DEVICE_COLOR_IMG_PORT, _msgHandler);
        _listenerColorImgThread.start();
        _listenerDepthImgThread = new ListenerThread(DEVICE_DEPTH_IMG_PORT, _msgHandler);
        _listenerDepthImgThread.start();

        // for time display
        _tv_timeDisplay_home = findViewById(R.id.textview_time_home);
        _tv_timeDisplay_wifi = findViewById(R.id.textview_time_wifi);
        _tv_timeDisplay_help = findViewById(R.id.textview_time_help);

        // double click event
        this._tv_timeDisplay_home.setOnClickListener(new DoubleClickListener() {
            @Override
            public void onDoubleClick(View v) {
                _tv_deviceConnectState.setText("");
                _tv_deviceMsg.setText("");
                _tv_rgbPath.setText("");
                _tv_depthPath.setText("");
                _tv_timeStamp.setText("");
                _tv_ax.setText("");
                _tv_ay.setText("");
                _tv_az.setText("");
                _tv_gx.setText("");
                _tv_gy.setText("");
                _tv_gz.setText("");
            }
        });
        // handler for time display
        _timeDisplayHandler = new Handler(Looper.getMainLooper()) {
            @RequiresApi(api = Build.VERSION_CODES.O)
            @Override
            public void handleMessage(Message msg) {
                String time = LocalDate.now().toString() + " " + LocalTime.now().toString();
                _tv_timeDisplay_wifi.setText(time);
                _tv_timeDisplay_home.setText(time);
                _tv_timeDisplay_help.setText(time);
            }
        };
        new Thread(() -> {
            while (true) {
                _timeDisplayHandler.sendEmptyMessage(0);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    public void getWifiAdmin() {
        this._wifiManager = (WifiManager) this.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
    }

    @SuppressLint("SetTextI18n")
    public void displayWifiInfo() {
        // get wifi info
        this._wifiInfo = this._wifiManager.getConnectionInfo();

        // display
        // --


        // --

        _tv_ssid.setText(this._wifiInfo.getSSID());
        Log.d("-----------", this._wifiInfo.toString());
        _tv_rssi.setText(String.valueOf(this._wifiInfo.getRssi()));
        _tv_speed.setText(this._wifiInfo.getLinkSpeed() + " Mbps");
        _tv_frequency.setText(this._wifiInfo.getFrequency() + " MHz");
        _tv_netId.setText(String.valueOf(this._wifiInfo.getNetworkId()));
        _tv_supState.setText(this._wifiInfo.getSupplicantState().toString());


        // if wifi is not turned on, set "_rightConnect" to "0" and return
        if (this._wifiManager.getWifiState() == WifiManager.WIFI_STATE_DISABLED) {
            this._targetWiFiConnected = 0;
            return;
        }

        // display ip address
        TextView hostIp = findViewById(R.id.textview_ip);
        hostIp.setText(intIP2StringIP(this._wifiInfo.getIpAddress()));

        TextView port = findViewById(R.id.textview_port);
        port.setText("msg(" + DEVICE_MSG_PORT + ") img(" + DEVICE_COLOR_IMG_PORT + ';' + DEVICE_DEPTH_IMG_PORT + ")");

        this.checkTargetWiFi();
    }

    public void checkTargetWiFi() {
        // if wifi is not turned on, check the up address
        Log.d("---checkTargetWiFi---", this._wifiInfo.getSSID());
        if (('\"' + _targetWiFiSSID + '\"').equals(this._wifiInfo.getSSID())) {
            // is the target wifi
            if (this._targetWiFiConnected != 1) {
                Toast.makeText(this, "successfully touch to " + _targetWiFiSSID, Toast.LENGTH_LONG).show();
                this._targetWiFiConnected = 1;
            }
        } else {
            // is not the target wifi
            if (this._targetWiFiConnected != -1) {
                Toast.makeText(this, "please connect to " + _targetWiFiSSID, Toast.LENGTH_LONG).show();
                this._targetWiFiConnected = -1;
            }
        }
    }

    public void registerWifiReceiver() {
        IntentFilter intentFilter = new IntentFilter();
        // wifi state
        intentFilter.addAction(WifiManager.WIFI_STATE_CHANGED_ACTION);
        // connect state
        intentFilter.addAction(WifiManager.NETWORK_STATE_CHANGED_ACTION);
        // register
        this.registerReceiver(_wifiStateReceiver, intentFilter);
    }

    public abstract static class DoubleClickListener implements View.OnClickListener {
        private static final long DOUBLE_TIME = 1000;
        private static long lastClickTime = 0;

        @Override
        public void onClick(View v) {
            long currentTimeMillis = System.currentTimeMillis();
            if (currentTimeMillis - lastClickTime < DOUBLE_TIME) {
                onDoubleClick(v);
            }
            lastClickTime = currentTimeMillis;
        }

        public abstract void onDoubleClick(View v);
    }

    public static class ConnectThread extends Thread {

        private final Socket socket;
        private final Handler handler;
        private final int _successfulMsgType;
        // for receive
        private InputStream inputStream;
        // for send
        private OutputStream outputStream;

        public ConnectThread(Socket socket, Handler handler, int successfulMsgType) {
            setName("ConnectThread");
            this.socket = socket;
            this.handler = handler;
            _successfulMsgType = successfulMsgType;
        }

        @Override
        public void run() {
            // if no socket
            if (socket == null) {
                return;
            }
            try {
                socket.setTcpNoDelay(true);
            } catch (SocketException e) {
                e.printStackTrace();
            }
            // send message for "connected"
            handler.sendEmptyMessage(MainActivity.DEVICE_CONNECTED);
            try {
                // get streams
                inputStream = socket.getInputStream();
                outputStream = socket.getOutputStream();
                byte[] buffer = new byte[2048];
                int bytes;
                // read stream data [get message]
                while (true) {
//                    Thread.sleep(10);
                    if (_successfulMsgType == MainActivity.GET_MSG_SUCCESS) {
                        bytes = inputStream.read(buffer);
                        if (bytes > 0) {
                            final byte[] data = new byte[bytes];
                            System.arraycopy(buffer, 0, data, 0, bytes);
                            Message message = Message.obtain();
                            message.what = _successfulMsgType;
                            Bundle bundle = new Bundle();
                            // get text message
                            bundle.putString("MSG", new String(data));
                            message.setData(bundle);
                            handler.sendMessage(message);
                        }
                    } else {
                        // for 'GET_COLOR_IMG_SUCCESS' and 'GET_DEPTH_IMG_SUCCESS'

                        // if size is not enough
                        int curSize = 0;
                        byte[] data = new byte[2048];
                        while (curSize < 2048) {
                            bytes = inputStream.read(buffer, 0, 2048 - curSize);
                            if (bytes > 0) {
                                System.arraycopy(buffer, 0, data, curSize, bytes);
                                curSize += bytes;
                            }
                        }
                        Message message = Message.obtain();
                        message.what = _successfulMsgType;
                        Bundle bundle = new Bundle();
                        // get image
                        bundle.putByteArray("MSG", data);
                        message.setData(bundle);
                        handler.sendMessage(message);
                    }

//                    bytes = inputStream.read(buffer);
//                    // has data
//                    if (bytes > 0) {
//                        final byte[] data = new byte[bytes];
//                        System.arraycopy(buffer, 0, data, 0, bytes);
//                        Message message = Message.obtain();
//                        message.what = _successfulMsgType;
//                        Bundle bundle = new Bundle();
//                        if (_successfulMsgType == MainActivity.GET_MSG_SUCCESS) {
//                            // get text message
//                            bundle.putString("MSG", new String(data));
//                        } else if (_successfulMsgType == MainActivity.GET_IMG_SUCCESS) {
//                            // get image
//                            bundle.putByteArray("MSG", data);
//                        }
//                        message.setData(bundle);
//                        handler.sendMessage(message);
//                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        /**
         * send data
         */
        public void sendData(String msg) {
            if (socket == null) {
                return;
            }
            try {
                socket.setTcpNoDelay(true);
            } catch (SocketException e) {
                e.printStackTrace();
            }
            try {
                outputStream = socket.getOutputStream();
                // write data [send message]
                outputStream.write(msg.getBytes());
                // send message for "send message success"
                Message message = Message.obtain();
                message.what = MainActivity.SEND_MSG_SUCCESS;
                Bundle bundle = new Bundle();
                bundle.putString("MSG", msg);
                message.setData(bundle);
                handler.sendMessage(message);
            } catch (IOException e) {
                e.printStackTrace();
                // send message for "send message error"
                Message message = Message.obtain();
                message.what = MainActivity.SEND_MSG_ERROR;
                Bundle bundle = new Bundle();
                bundle.putString("MSG", msg);
                message.setData(bundle);
                handler.sendMessage(message);
            }
        }
    }

    public static class ListenerThread extends Thread {

        private final Handler handler;
        private ServerSocket serverSocket = null;
        private Socket socket;

        public ListenerThread(int port, Handler handler) {
            setName("ListenerThread");
            this.handler = handler;
            try {
                serverSocket = new ServerSocket(port);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void run() {
            while (true) {
                try {
                    if (serverSocket != null) {
                        // if no connection, then this method blocks here
                        socket = serverSocket.accept();
                    }
                    // send message for "device connecting"
                    Message message = Message.obtain();
                    message.what = MainActivity.DEVICE_CONNECTING;
                    handler.sendMessage(message);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        public Socket getSocket() {
            return socket;
        }
    }

    static class DataItem {
        float[] _values;

        DataItem(float[] values) {
            this._values = values;
        }
    }

    class WifiStateBroadcastReceive extends BroadcastReceiver {

        @Override
        public void onReceive(Context context, Intent intent) {
            // wifi state
            int wifiState = intent.getIntExtra(WifiManager.EXTRA_WIFI_STATE, 0);
            switch (wifiState) {
                case WifiManager.WIFI_STATE_DISABLED:
                    Toast.makeText(context, "please turn on WiFi", Toast.LENGTH_SHORT).show();
                    break;
                case WifiManager.WIFI_STATE_ENABLED:
                    Toast.makeText(context, "WiFi turned on", Toast.LENGTH_SHORT).show();
                    displayWifiInfo();
                    break;
                case WifiManager.WIFI_STATE_UNKNOWN:
                    break;
            }
            // wifi connection state
            if (intent.getAction().equals(WifiManager.NETWORK_STATE_CHANGED_ACTION)) {
                displayWifiInfo();
            }
        }
    }
}