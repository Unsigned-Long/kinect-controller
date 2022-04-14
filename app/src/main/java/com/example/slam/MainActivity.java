package com.example.slam;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.tabs.TabLayout;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.time.LocalDate;
import java.time.LocalTime;

public class MainActivity extends AppCompatActivity {
    public static final int DEVICE_CONNECTING = 1;
    public static final int DEVICE_CONNECTED = 2;
    public static final int SEND_MSG_SUCCESS = 3;
    public static final int SEND_MSG_ERROR = 4;
    public static final int GET_MSG_SUCCESS = 6;

    // this is the port
    public static final int DEVICE_PORT = 3890;
    private static final String _targetWiFiSSID = "slam";
    private static final String _targetHostName = "slam-device";

//    private static final String _targetWiFiSSID = "Juster";
//    private static final String _targetHostName = "whu-ubuntu";

    // message flags
    private static final char COLOR_DEPTH_TIME = '0';
    private static final char INFO_MESSAGE_TEXTVIEW = '1';
    private static final char INFO_MESSAGE_TOAST = '2';
    // wifi
    private WifiManager _wifiManager;
    private WifiInfo _wifiInfo;
    //  "0": wifi not turned on
    //  "1": wifi turned on and connect to target wifi
    // "-1": wifi turned on but not connect to target wifi
    private int _targetWiFiConnected;
    private WifiStateBroadcastReceive _wifiStateReceiver;
    // textview [wifi info]
    private TextView _textViewDeviceConnectState;
    private TextView _timeDisplay_wifi;
    private TextView _timeDisplay_home;
    private TextView _timeDisplay_help;
    private TextView _ssid;
    private TextView _rssi;
    private TextView _speed;
    private TextView _frequency;
    private TextView _netId;
    private TextView _deviceMsg;
    private TextView _rgbPath;
    private TextView _depthPath;
    private TextView _timeStamp;
    private TextView _supState;
    private ImageView _letter_s;
    private ImageView _letter_l;
    private ImageView _letter_a;
    private ImageView _letter_m;
    private Handler _timeDisplayHandler;
    // frame
    private FrameLayout _frameSlam, _frameWifi, _frameHelp;
    // tcp
    private ConnectThread _connectThread;
    private ListenerThread _listenerThread;
    private Handler _msgHandler;

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

    public void handleReceivedMsg(String str) {

        if (str.charAt(0) == INFO_MESSAGE_TEXTVIEW) {
            _deviceMsg.setText(str.substring(1));
        } else if (str.charAt(0) == COLOR_DEPTH_TIME) {
            String[] strAry = str.substring(1).split(":");
            _rgbPath.setText(strAry[0]);
            _depthPath.setText(strAry[1]);
            _timeStamp.setText(strAry[2]);
        } else if (str.charAt(0) == INFO_MESSAGE_TOAST) {
            _deviceMsg.setText(str.substring(1));
            Toast.makeText(MainActivity.this, str.substring(1), Toast.LENGTH_SHORT).show();
        }

        Log.d("---", str);
    }

    @SuppressLint("SetTextI18n")
    public void initVariables() {
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
        _ssid = findViewById(R.id.textview_ssid);
        _rssi = findViewById(R.id.textview_rssi);
        _speed = findViewById(R.id.textview_speed);
        _frequency = findViewById(R.id.textview_frequency);
        _netId = findViewById(R.id.textview_netId);
        _deviceMsg = findViewById(R.id.textview_deviceMsg);
        _rgbPath = findViewById(R.id.textview_rgb);
        _depthPath = findViewById(R.id.textview_depth);
        _timeStamp = findViewById(R.id.textview_imgTime);
        _supState = findViewById(R.id.textview_supstate);

        // images
        _letter_s = findViewById(R.id.img_s);
        _letter_l = findViewById(R.id.img_l);
        _letter_a = findViewById(R.id.img_a);
        _letter_m = findViewById(R.id.img_m);

        // tcp
        this._textViewDeviceConnectState = findViewById(R.id.text_state);

        _msgHandler = new Handler(Looper.getMainLooper()) {
            @SuppressLint("SetTextI18n")
            @Override
            public void handleMessage(Message msg) {
                switch (msg.what) {
                    case DEVICE_CONNECTING:
                        _connectThread = new ConnectThread(_listenerThread.getSocket(), _msgHandler);
                        _connectThread.start();
                        break;
                    case DEVICE_CONNECTED:
                        _textViewDeviceConnectState.setText("Device Connected Successfully");
                        break;
                    case SEND_MSG_SUCCESS:
                        _textViewDeviceConnectState.setText("Message Sent Successfully [" + msg.getData().getString("MSG") + "]");
                        break;
                    case SEND_MSG_ERROR:
                        _textViewDeviceConnectState.setText("Failed To Send Message [" + msg.getData().getString("MSG") + "]");
                        break;
                    case GET_MSG_SUCCESS:
                        String message = msg.getData().getString("MSG");
                        handleReceivedMsg(message);
                        break;
                    default:
                }
            }
        };

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
                        Socket socket = new Socket(_targetHostName, DEVICE_PORT);
                        // construct a new connect thread
                        _connectThread = new ConnectThread(socket, _msgHandler);
                        // start connect
                        _connectThread.start();

                    } catch (IOException e) {
                        e.printStackTrace();
                        runOnUiThread(() -> _textViewDeviceConnectState.setText("Connect failed, Server Not Running"));
                    }
                }).start();
            }
        });

        Button _btnStart = findViewById(R.id.button_start);
        _btnStart.setOnClickListener(view -> {
            if (_connectThread != null) {
                new Thread(() -> _connectThread.sendData("start")).start();

                Animation animation = AnimationUtils.loadAnimation(MainActivity.this, R.anim.running);

                _letter_s.startAnimation(animation);
                _letter_l.startAnimation(animation);
                _letter_a.startAnimation(animation);
                _letter_m.startAnimation(animation);

            } else {
                Toast.makeText(MainActivity.this, "device isn't connected", Toast.LENGTH_SHORT).show();
            }
        });

        Button _btnStop = findViewById(R.id.button_stop);
        _btnStop.setOnClickListener(view -> {
            if (_connectThread != null) {
                new AlertDialog.Builder(this, R.style.dialogStyle)
                        .setTitle("Information")
                        .setMessage("Are you sure to stop collection?")
                        .setIcon(R.drawable.exc)
                        .setPositiveButton("Yes", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                new Thread(() -> _connectThread.sendData("stop")).start();
                                _letter_s.clearAnimation();
                                _letter_l.clearAnimation();
                                _letter_a.clearAnimation();
                                _letter_m.clearAnimation();
                            }
                        })
                        .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                            }
                        })
                        .create()
                        .show();

            } else {
                Toast.makeText(MainActivity.this, "device isn't connected", Toast.LENGTH_SHORT).show();
            }
        });

        _listenerThread = new ListenerThread(DEVICE_PORT, _msgHandler);
        _listenerThread.start();

        // for time display
        _timeDisplay_home = findViewById(R.id.textview_time_home);
        this._timeDisplay_home.setOnClickListener(new DoubleClickListener() {
            @Override
            public void onDoubleClick(View v) {
                _textViewDeviceConnectState.setText("");
                _deviceMsg.setText("");
                _rgbPath.setText("");
                _depthPath.setText("");
                _timeStamp.setText("");
            }
        });
        _timeDisplay_wifi = findViewById(R.id.textview_time_wifi);
        _timeDisplay_help = findViewById(R.id.textview_time_help);
        _timeDisplayHandler = new Handler(Looper.getMainLooper()) {
            @RequiresApi(api = Build.VERSION_CODES.O)
            @Override
            public void handleMessage(Message msg) {
                String time = LocalDate.now().toString() + " " + LocalTime.now().toString();
                _timeDisplay_wifi.setText(time);
                _timeDisplay_home.setText(time);
                _timeDisplay_help.setText(time);
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

        _ssid.setText(this._wifiInfo.getSSID());
        Log.d("-----------", this._wifiInfo.toString());
        _rssi.setText(String.valueOf(this._wifiInfo.getRssi()));
        _speed.setText(this._wifiInfo.getLinkSpeed() + " Mbps");
        _frequency.setText(this._wifiInfo.getFrequency() + " MHz");
        _netId.setText(String.valueOf(this._wifiInfo.getNetworkId()));
        _supState.setText(this._wifiInfo.getSupplicantState().toString());


        // if wifi is not turned on, set "_rightConnect" to "0" and return
        if (this._wifiManager.getWifiState() == WifiManager.WIFI_STATE_DISABLED) {
            this._targetWiFiConnected = 0;
            return;
        }

        // display ip address
        TextView hostIp = findViewById(R.id.textview_ip);
        hostIp.setText(intIP2StringIP(this._wifiInfo.getIpAddress()));

        TextView port = findViewById(R.id.textview_port);
        port.setText(String.valueOf(DEVICE_PORT));

        this.checkTargetWiFi();
    }

    public void checkTargetWiFi() {
        // if wifi is not turned on, check the up address
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
        // for receive
        private InputStream inputStream;
        // for send
        private OutputStream outputStream;

        public ConnectThread(Socket socket, Handler handler) {
            setName("ConnectThread");
            this.socket = socket;
            this.handler = handler;

        }

        @Override
        public void run() {
            // if no socket
            if (socket == null) {
                return;
            }
            // send message for "connected"
            handler.sendEmptyMessage(MainActivity.DEVICE_CONNECTED);
            try {
                // get streams
                inputStream = socket.getInputStream();
                outputStream = socket.getOutputStream();

                byte[] buffer = new byte[1024];
                int bytes;
                // read stream data [get message]
                while (true) {
                    Thread.sleep(10);
                    bytes = inputStream.read(buffer);
                    // has data
                    if (bytes > 0) {
                        final byte[] data = new byte[bytes];
                        System.arraycopy(buffer, 0, data, 0, bytes);
                        Message message = Message.obtain();
                        // send message for "get message"
                        message.what = MainActivity.GET_MSG_SUCCESS;
                        Bundle bundle = new Bundle();
                        bundle.putString("MSG", new String(data));
                        message.setData(bundle);
                        handler.sendMessage(message);
                    }
                }
            } catch (IOException | InterruptedException e) {
                e.printStackTrace();
            }
        }

        /**
         * send data
         */
        public void sendData(String msg) {
            if (outputStream != null) {
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