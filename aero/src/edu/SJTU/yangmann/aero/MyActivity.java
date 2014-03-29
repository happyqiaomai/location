package edu.SJTU.yangmann.aero;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.FloatMath;
import android.util.Log;
import android.view.View;
import android.widget.*;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.RandomAccessFile;


import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.sql.Time;
import java.util.Date;

public class MyActivity extends Activity implements SensorEventListener {

    private SensorManager mSensorManager;
    private Sensor mAccelerometer = null; //线性加速度
    private Sensor mGravitySensor = null;
    private Sensor mMagneticSensor = null;
    private Sensor mRotAcc = null; //加速度
    private Sensor mRotVel = null;  //角速度
    private float mAccel;
    private float mAccelCurrent;
    private float mAccelLast;
    private float vaccx, vaccy, vaccz, vaccxnew, vaccynew, vaccznew,
            vworldx, vworldy, vworldz,
            vgrax, vgray, vgraz, vgraxnew, vgraynew, vgraznew,
            vmagx, vmagy, vmagz, vmagxnew, vmagynew, vmagznew;

    private float[] RotationMatrix = {(float) 1, (float) 0, (float) 0,
            (float) 0, (float) 1, (float) 0, (float) 0, (float) 0, (float) 1};
    private float[] GravityVector = {(float) 0, (float) 0, (float) 9.8};
    private float[] MagneticVector = {(float) 0, (float) 0, (float) 0};
    private float[] WorldVector = {(float) 0, (float) 0, (float) 0};
    private float[] AccelerVector = {(float) 0, (float) 0, (float) 0};

    float[] accelerometerValues = new float[3];  //加速度

    float[] magneticFieldValues = new float[3];   //地磁 为了求orientation

    float[] anglespeedValues = new float[3];   //绕轴角速度



    private static final String TAG = "sensor";




    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];  //改
    private float timestamp;
    private final float[] speed = new float[3];
    private final float[] loc = new float[3];
    private final float[] Fspeed = new float[3];
    private final float[] Floc = new float[3];

    private TextView TextViewX = null;    //文本框
    private TextView TextViewV = null;    //文本框
    private TextView TextViewA = null;    //文本框

    private TextView TextViewFX = null;    //文本框
    private TextView TextViewFV = null;    //文本框
    private TextView TextViewFA = null;    //文本框


    private TextView TextViewYa = null;    //文本框
    private TextView TextViewP = null;    //文本框
    private TextView TextViewR = null;    //文本框


    private Button bt1 = null;
    private ToggleButton bt2 = null;

    private String fileName;

    private int state; /// 0:显示模式   1：记录模式
    private float dT, dT2;
    private int counter = 0, accounter = 0, decounter = 0, accnumber = 0;
    private static final float hz = (float) 2.0, pi = (float) 3.14159265, average = (float) 0.9, threshacc = (float) 1.3;
    private float ws = 0, threshwindow = (float) 16.0, accwindow = 0, mSin, mCos, accvalue;    //threshwindow=average*window
    private static final int fs = 16, wlen = 50, window = 16;
    private int slen = 0, judgetimecounter = 0, notaccounter = 0;
    private float[] sinc;
    private boolean startjudge = false;     //作用是什么？
    private float[] worldxfiltered, worldyfiltered, worldxraw, worldyraw, worldzraw, worldzfiltered;

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        dT2 = 0;
        dT = 0;
        state = 0;

        ws = (float) (2 * pi * hz / ((float) fs));
        slen = 2 * (int) (fs / hz);
        sinc = new float[slen];
        worldxfiltered = new float[wlen];
        worldxraw = new float[wlen];
        worldyfiltered = new float[wlen];
        worldyraw = new float[wlen];
        worldzfiltered = new float[wlen];
        worldzraw = new float[wlen];
        for (int i = 0; i < slen; i++) {
            if (i - slen / 2 != 0) {
                sinc[i] = (float) (1 / pi * Math.sin(ws * (i - slen / 2)) / (i - slen / 2));
            } else {
                sinc[i] = ws / pi;
            }
        }
        threshwindow = window * average;


        //// TODO 以上来自以前
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mAccel = 0.00f;
        mAccelCurrent = SensorManager.GRAVITY_EARTH;
        mAccelLast = SensorManager.GRAVITY_EARTH;

        mRotVel =  mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mRotAcc =  mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mGravitySensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        mMagneticSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        Time tmp = new Time(System.currentTimeMillis()); //dddd
        //tmp.setToNow();

        fileName = "LOC"+tmp.toString()+".txt";
        Log.i(TAG, fileName);


        Toast.makeText(getApplicationContext(), "当前系统时间为： " + fileName, Toast.LENGTH_LONG);

        View.OnClickListener listener = null;                  //声明监听器

        TextViewX = (TextView) findViewById(R.id.textViewx);    //文本框
        TextViewV = (TextView) findViewById(R.id.textViewv);    //文本框
        TextViewA = (TextView) findViewById(R.id.textViewa);    //文本框

        TextViewFX = (TextView) findViewById(R.id.textViewFx);    //文本框
        TextViewFV = (TextView) findViewById(R.id.textViewFv);    //文本框
        TextViewFA = (TextView) findViewById(R.id.textViewFa);    //文本框

        TextViewYa = (TextView) findViewById(R.id.textViewYa);    //文本框
        TextViewP = (TextView) findViewById(R.id.textViewP);    //文本框
        TextViewR = (TextView) findViewById(R.id.textViewR);    //文本框

        bt1 = (Button) findViewById(R.id.button1);                            //按钮1

        bt2 = (ToggleButton) findViewById(R.id.toggleButton);   //切换按钮

        bt1.setOnClickListener(listener = new View.OnClickListener() {                //设置监听器
            @Override
            public void onClick(View v) {
                // TODO Auto-generated method stub
                //TextViewX.setText("System time in nanoseconds: " + Long.toString(System.nanoTime()));				//设置文本   System.nanoTime() 挺好用 但是貌似用不到
                speed[0] = 0;
                Fspeed[0] = 0;
                speed[1] = 0;
                Fspeed[1] = 0;
                speed[2] = 0;
                Fspeed[2] = 0;
                loc[0] = 0;
                Floc[0] = 0;
                loc[1] = 0;
                Floc[1] = 0;
                loc[2] = 0;
                Floc[2] = 0;

                //// TODO: Add a Timer for 30 seconds to get better calibration.
            }
        });

        bt2.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                //To change body of implemented methods use File | Settings | File Templates.

                if (isChecked) {
                    state = 1;
                    //改变sensor状态 暂时不可用/////////////////////
                } else {
                    state = 0;
                }
            }
        });

    }

    @Override
    public void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);  //为何？ 不可用FASTEST
        mSensorManager.registerListener(this, mGravitySensor, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagneticSensor, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotAcc, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotVel, SensorManager.SENSOR_DELAY_UI);

    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    private float[] getnewvector(float[] x, float[] y) {
        float[] Matrix = new float[3];
        Matrix[0] = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        Matrix[1] = x[3] * y[0] + x[4] * y[1] + x[5] * y[2];
        Matrix[2] = x[6] * y[0] + x[7] * y[1] + x[8] * y[2];
        return Matrix;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)

            magneticFieldValues = event.values;

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)

            accelerometerValues = event.values;

        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {

            anglespeedValues = event.values;

        }

        calculateOrientation();
        showSpeed();

        if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            float[] mGravity = event.values.clone();
            mAccelLast = mAccelCurrent;
            mAccelCurrent = FloatMath.sqrt(mGravity[0] * mGravity[0] + mGravity[1] * mGravity[1] + mGravity[2] * mGravity[2]);
            mAccel = mAccel * 0.9f + mAccelCurrent - mAccelLast;
            /*Log.v("mAccel", "[Gx]" + mGravity[0] + "\t[Gy]" + mGravity[1] + "\t[Gz]" +
                    mGravity[2] + "\tThread Time[ms]");*/

            // Tried the elapsedRealtimeNanos() but didn't work. Moved it to the button callback.
            if (mAccel > 3) {
                Log.wtf("mAccel", "SHAKE!!!!");
            }

            //Log.e("COUNT", " " + counter + "STAMP " + timestamp);
            // 积分的部分 NS2S
            if (timestamp != 0) {
                dT = (event.timestamp - timestamp) * NS2S;  //////////////////////////////
                TextViewR.setText((Float.toString(dT)));

                dT2 += (event.timestamp - timestamp) * NS2S;                            // Timer for Filtered data
                speed[0] += event.values[0] * dT;
                speed[1] += event.values[1] * dT;
                speed[2] += event.values[2] * dT;

                loc[0] += speed[0] * dT;
                loc[1] += speed[1] * dT;
                loc[2] += speed[2] * dT;

                TextViewA.setText("Acc X: " + event.values[0] + "\nY: " + event.values[1] + "\nZ: " + event.values[2]);
                TextViewV.setText("Vel X: " + speed[0] + "\nY: " + speed[1] + "\nZ: " + speed[2]);
                TextViewX.setText("Accuracy: " + event.accuracy + "\nLoc X: " + loc[0] + "\nY: " + loc[1] + "\nZ: " + loc[2]);


                // 来自以前
                // 滤波方法大致如下： 二阶低通滤波

                float Q = 1/FloatMath.sqrt(2); //品质因子
                float iQ = 1.0f / Q;
                float fc = 1000;


                float K = (float) Math.tan((pi) * fc * dT);
                float iD = 1.0f / (K*K + K*iQ + 1);
                float a0 = K*K*iD;
                float a1 = 2.0f * a0;
                float b1 = 2.0f*(K*K - 1)*iD;
                float b2 = (K*K - K*iQ + 1)*iD;



                //float y = (x + x2) + x1*a1 - y1*b1 - y2*b2; //过滤后值


                AccelerVector[0] = vaccx = (vaccx + event.values[0]) / 2;
                AccelerVector[1] = vaccy = (vaccy + event.values[1]) / 2;
                AccelerVector[2] = vaccz = (vaccz + event.values[2]) / 2;

                //Log.e("COUNT", "" + counter);
                if (counter == 1) {    //  50/3 hz   Not Happening
                    // get world vector
                    SensorManager.getRotationMatrix(RotationMatrix, null,
                            GravityVector, MagneticVector);
                    WorldVector = getnewvector(RotationMatrix, AccelerVector);  //// TODO 非常重要的 但为什么会变成
                    // store world vect
                    for (int i = 1; i < wlen; i++) {
                        worldxraw[i] = worldxraw[i - 1];
                        worldxfiltered[i] = worldxfiltered[i - 1];
                        worldyraw[i] = worldyraw[i - 1];
                        worldyfiltered[i] = worldyfiltered[i - 1];
                        worldzraw[i] = worldzraw[i - 1];
                        worldzfiltered[i] = worldzfiltered[i - 1];
                    }
                    worldxraw[0] = vworldx = WorldVector[0];
                    worldyraw[0] = vworldy = WorldVector[1];
                    worldzraw[0] = vworldz = WorldVector[2];
                    //vworldz=WorldVector[2];
                    // start sinc filter
                    worldxfiltered[0] = worldyfiltered[0] = worldzfiltered[0] = 0;
                    for (int i = 0; i < slen; i++) {
                        worldxfiltered[0] += sinc[i] * worldxraw[i];
                        worldyfiltered[0] += sinc[i] * worldyraw[i];
                        worldzfiltered[0] += sinc[i] * worldzraw[i];
                    }

                    Fspeed[0] += worldxfiltered[0] * dT2;
                    Fspeed[1] += worldyfiltered[0] * dT2;
                    Fspeed[2] += worldzfiltered[0] * dT2;

                    Floc[0] += Fspeed[0] * dT2;
                    Floc[1] += Fspeed[1] * dT2;
                    Floc[2] += Fspeed[2] * dT2;

                    /// 输出结果的部分
                   /* /Log.v("mLocation", "[x]" + loc[0] + "\t[y]" + loc[1] + "\t[z]" +
                            loc[2] + "\tDelta Time[s] " + dT);*/
                    TextViewFA.setText("Acc X: " + worldxfiltered[0] +
                            "\nY: " + worldyfiltered[0] + "\nZ: " + worldzfiltered[0]);
                    TextViewFV.setText("Vel X: " + Fspeed[0] + "\nY: " + Fspeed[1] + "\nZ: " + Fspeed[2]);
                    TextViewFX.setText("------Filtered Data------\nLoc X: " + Floc[0] + "\nY: " + Floc[1] + "\nZ: " + Floc[2]);

                    if (state == 1) {
                        float fvel_all = (Fspeed[0] * Fspeed[0] + Fspeed[1] * Fspeed[1] + Fspeed[2] * Fspeed[2]);
                        fvel_all = FloatMath.sqrt(fvel_all);
                        float vel_all = FloatMath.sqrt(speed[0] * speed[0] + speed[1] * speed[1] + speed[2] * speed[2]);
                        save2file(Long.toString(System.currentTimeMillis()) + "\t" + Float.toString(fvel_all) + "\t" + Float.toString(vel_all) + "\n");
                    }

                    dT2 = 0;

                } else if (counter >= 2) {
                    counter = 0;
                } else {
                    counter++;
                    //Log.e("COUNT000",""+counter);
                }
            }
            timestamp = event.timestamp;

            // End of the ACC Sensor

        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            vgraxnew = event.values[0];
            vgraynew = event.values[1];
            vgraznew = event.values[2];
            GravityVector[0] = vgrax = (vgrax + vgraxnew) / 2;
            GravityVector[1] = vgray = (vgray + vgraynew) / 2;
            GravityVector[2] = vgraz = (vgraz + vgraznew) / 2;
            //Log.v("mGravity", "X: " + GravityVector[0]);
           /* tvgrax.setText("X: " + vgrax);
            tvgray.setText("Y: " + vgray);
            tvgraz.setText("Z: " + vgraz);*/
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            vmagxnew = event.values[0];
            vmagynew = event.values[1];
            vmagznew = event.values[2];
            MagneticVector[0] = vmagx = (vmagx + vmagxnew) / 2;
            MagneticVector[1] = vmagy = (vmagy + vmagynew) / 2;
            MagneticVector[2] = vmagz = (vmagz + vmagznew) / 2;
            //Log.v("mMag", "X: " + MagneticVector[0]);
            /*tvmagx.setText("X: " + vmagx);
            tvmagy.setText("Y: " + vmagy);
            tvmagz.setText("Z: " + vmagz);*/
        }

    }

    private  void calculateOrientation() {

        float[] values = new float[3];

        float[] R = new float[9];

        SensorManager.getRotationMatrix(R, null, accelerometerValues, magneticFieldValues);

        SensorManager.getOrientation(R, values);



        // 要经过一次数据格式的转换，转换为度

        values[0] = (float) Math.toDegrees(values[0]);

        values[1] = (float) Math.toDegrees(values[1]);

        values[2] = (float) Math.toDegrees(values[2]);


        TextViewYa.setText("Yaw: " + values[0] + "\nPitch: " +values[1] + "\nRoll: " + values[2]);

        Log.i(TAG, values[0]+"");


        if(values[0] >= -5 && values[0] < 5){

            Log.i(TAG, "正北");

        }

        else if(values[0] >= 5 && values[0] < 85){

            Log.i(TAG, "东北");

        }

        else if(values[0] >= 85 && values[0] <=95){

            Log.i(TAG, "正东");

        }

        else if(values[0] >= 95 && values[0] <175){

            Log.i(TAG, "东南");

        }

        else if((values[0] >= 175 && values[0] <= 180) || (values[0]) >= -180 && values[0] < -175){

            Log.i(TAG, "正南");

        }

        else if(values[0] >= -175 && values[0] <-95){

            Log.i(TAG, "西南");

        }

        else if(values[0] >= -95 && values[0] < -85){

            Log.i(TAG, "正西");

        }

        else if(values[0] >= -85 && values[0] <-5){

            Log.i(TAG, "西北");

        }

    }

    public void showSpeed(){
        TextViewP.setText("Xvel: " + (float) Math.toDegrees(anglespeedValues[0]) + "\nYvel: " +(float) Math.toDegrees(anglespeedValues[1]) + "\nZvel: " + (float) Math.toDegrees(anglespeedValues[2]));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    private void save2file(String a){

        try {

            File file = new File("/sdcard/"+fileName);
            if (!file.exists()){
                file.createNewFile();}

            // 打开一个随机访问文件流，按读写方式
            RandomAccessFile randomFile = new RandomAccessFile("/sdcard/"+fileName, "rw");
            // 文件长度，字节数
            long fileLength = randomFile.length();
            // 将写文件指针移到文件尾。
            randomFile.seek(fileLength);
            randomFile.writeBytes(a);
            //Log.e("!","!!");
            randomFile.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }
}