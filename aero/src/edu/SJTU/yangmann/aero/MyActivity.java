package edu.SJTU.yangmann.aero;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.FloatMath;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.sql.Time;

import jama.Matrix;
import jkalman.JKalman;

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
    private float[] WorldVector_last = {(float) 0, (float) 0, (float) 0};
    private float[] WorldVector_current = {(float) 0, (float) 0, (float) 0};
    private float[] AccelerVector_raw = {(float) 0, (float) 0, (float) 0};
    private float[] AccelerVector_filtered = {(float) 0, (float) 0, (float) 0};


    float[] accelerometerValues = new float[3];  //加速度

    float[] magneticFieldValues = new float[3];   //地磁传感器数据 为了求orientation

    float[] anglespeedValues = new float[3];   //绕轴角速度



    private static final String TAG = "sensor";

    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];  //改
    private double timestamp;
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


    private TextView TextViewYa = null;    //文本框  显示方向
    private TextView TextViewP = null;    //文本框  显示沿轴加速度
    private TextView TextViewR = null;    //文本框  过滤后沿轴加速度


    private Button bt1 = null;
    private ToggleButton bt2 = null;

    private String fileName;
    private JKalman kalman_ori;
    private Matrix state_ori;
    private Matrix state_corrected_ori;
    private Matrix measurement_ori;
    private JKalman kalman_vel;
    private Matrix state_vel;
    private Matrix state_corrected_vel;
    private Matrix measurement_vel;

    private int state; /// 0:显示模式   1：记录模式

    private int counter = 0, accounter = 0, decounter = 0, accnumber = 0;
    private static final float hz = (float) 2.0, pi = (float) 3.14159265, average = (float) 0.9, threshacc = (float) 1.3;
    private float ws = 0, threshwindow = (float) 16.0, accwindow = 0, mSin, mCos, accvalue;    //threshwindow=average*window
    private static final int fs = 16, wlen = 50, window = 16;
    private int slen = 0, judgetimecounter = 0, notaccounter = 0;
    private float[] sinc;
    float[] values = new float[3]; // orientation

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);


        state = 0;


        threshwindow = window * average;

        //Kalman filter for orientation & angular rate
        try {
            kalman_ori = new JKalman(6, 3);

            state_ori = new Matrix(6, 1); // state [x, y, z, dx, dy, dz]
            state_corrected_ori = new Matrix(6, 1);
            measurement_ori = new Matrix(3, 1); // measurement [dx, dy, dz]
            double[][] tr = {
                    {1, 0, 0, 1, 0, 0},
                    {0, 1, 0, 0, 1, 0},
                    {0, 0, 1, 0, 0, 1},
                    {1, 0, 0, 0, 0, 0},
                    {0, 1, 0, 0, 0, 0},
                    {0, 0, 1, 0, 0, 0}
            };
            kalman_ori.setTransition_matrix(new Matrix(tr));
            kalman_ori.setError_cov_post(kalman_ori.getError_cov_post().identity());
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        try {
            kalman_vel = new JKalman(6, 3);

            state_vel = new Matrix(6, 1); // state [x, y, z, dx, dy, dz]
            state_corrected_vel = new Matrix(6, 1);
            measurement_vel = new Matrix(3, 1); // measurement [dx, dy, dz]
            double[][] tr = {
                    {1, 0, 0, 1, 0, 0},
                    {0, 1, 0, 0, 1, 0},
                    {0, 0, 1, 0, 0, 1},
                    {1, 0, 0, 0, 0, 0},
                    {0, 1, 0, 0, 0, 0},
                    {0, 0, 1, 0, 0, 0}
            };
            kalman_vel.setTransition_matrix(new Matrix(tr));
            kalman_vel.setError_cov_post(kalman_vel.getError_cov_post().identity());
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }


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

        fileName = "LOC"+tmp.toString().substring(0,2)+"-"+tmp.toString().substring(3,5)+"-"+tmp.toString().substring(6,8)+".txt";
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
                //待加一个校准
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
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            magneticFieldValues = event.values;
        }

        else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            accelerometerValues = event.values;
        }

        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            //记得转化为world
            measurement_vel.set(0, 0, event.values[0]);
            measurement_vel.set(1, 0, event.values[1]);
            measurement_vel.set(2, 0, event.values[2]);
            state_vel = kalman_vel.Predict();
            state_corrected_vel = kalman_vel.Correct(measurement_vel);
            TextViewR.setText("------Kalman Data------\n"
                    +"aX\t" + state_corrected_vel.get(0, 0) + "\n"
                    +"aY\t"+ state_corrected_vel.get(1, 0) + "\n"
                    +"aZ\t"+ state_corrected_vel.get(2, 0) + "\n"
                    +"vX\t"+ state_corrected_vel.get(3, 0) + "\n"
                    +"vY\t"+ state_corrected_vel.get(4, 0) + "\n"
                    +"vZ\t"+ state_corrected_vel.get(5, 0) + "\n");

            anglespeedValues = event.values;


        }



        else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
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
                final double dT = (event.timestamp - timestamp)* NS2S;  //////////////////////////////
                Log.e("COUNT", "" + timestamp +" new: " + event.timestamp);




                // 滤波参数
                // 滤波方法大致如下： 二阶低通滤波
//
//                float Q = 1/FloatMath.sqrt(2); //品质因子
//                float iQ = 1.0f / Q;
                float fc = 1000;


//                float K = (float) Math.tan((pi) * fc * dT);
//                float iD = 1.0f / (K*K + K*iQ + 1);
//                float a0 = K*K*iD;
//                float a1 = 2.0f * a0;
//                float b1 = 2.0f*(K*K - 1)*iD;
//                float b2 = (K*K - K*iQ + 1)*iD;



                //float y = (x + x2) + x1*a1 - y1*b1 - y2*b2; //过滤后值 x1，y1 当前值，x2，y2前一时刻的值。  待实现
                float tau = 1 / (2* pi * fc);
//
//
//
                //float alpha = (float)( dT / (dT * tau));
                float alpha =(float) 0.2;
                Log.i(TAG, "alpha: "+Double.toString(alpha));
                Log.i(TAG, "time: "+Double.toString(dT));
//
//                filter1Out = filter1Out + (alpha * (input - filter1Out));
//                filter2Out = filter2Out + (alpha * (filter1Out - filter2Out));


                AccelerVector_raw[0] = event.values[0];
                AccelerVector_raw[1] = event.values[1];
                AccelerVector_raw[2] = event.values[2];

                //Log.e("COUNT", "" + counter);
                if (counter == 1) {    //  50/3 hz   Not Happening
                    // get world vector
                    SensorManager.getRotationMatrix(RotationMatrix, null,
                            GravityVector, MagneticVector);
                    WorldVector_last = WorldVector_current;
                    WorldVector_current = getnewvector(RotationMatrix, AccelerVector_raw);  //// TODO 非常重要的 但为什么会变成
                    AccelerVector_filtered[0] = WorldVector_last[0] + alpha * (WorldVector_current[0] - WorldVector_last[0]);
                    AccelerVector_filtered[0] = AccelerVector_filtered[0] + alpha * (WorldVector_current[0] - AccelerVector_filtered[0]);
                    AccelerVector_filtered[1] = WorldVector_last[1] + alpha * (WorldVector_current[1] - WorldVector_last[1]);
                    AccelerVector_filtered[1] = AccelerVector_filtered[1] + alpha * (WorldVector_current[1] - AccelerVector_filtered[1]);
                    AccelerVector_filtered[2] = WorldVector_last[2] + alpha * (WorldVector_current[2] - WorldVector_last[2]);
                    AccelerVector_filtered[2] = AccelerVector_filtered[2] + alpha * (WorldVector_current[2] - AccelerVector_filtered[2]);

                    TextViewA.setText("------Raw Data------\nAcc X: " + WorldVector_current[0] +
                            "\nY: " + WorldVector_current[1] + "\nZ: " + WorldVector_current[2]);
                    TextViewFA.setText("------Filtered Data------\nAcc X: " + AccelerVector_filtered[0] +
                            "\nY: " + AccelerVector_filtered[1] + "\nZ: " + AccelerVector_filtered[2]);

                    if (state == 1) {
                        save2file(Long.toString(System.currentTimeMillis()) + "\t" + WorldVector_current[0] + "\t" + WorldVector_current[1]  + "\t" + WorldVector_current[2] +
                                "\t"+AccelerVector_filtered[0]+"\t"+AccelerVector_filtered[1]+"\t"+AccelerVector_filtered[2]+"\n"); //改成记录所有数据  accx,accy,accz,yaw,row,pitch,angx,angy,angz. filtered...
                    }



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
        calculateOrientation(); //计算方向
        showSpeed(); //显示沿各轴角加速度

    }

    private  void calculateOrientation() {



        float[] R = new float[9];

        SensorManager.getRotationMatrix(R, null, accelerometerValues, magneticFieldValues);

        SensorManager.getOrientation(R, values);



        // 要经过一次数据格式的转换，转换为度

        values[0] = (float) Math.toDegrees(values[0]);

        values[1] = (float) Math.toDegrees(values[1]);

        values[2] = (float) Math.toDegrees(values[2]);


        TextViewYa.setText("Yaw: " + values[0] + "\nPitch: " +values[1] + "\nRoll: " + values[2]); //分别是沿z轴 x轴 y轴


        //Log.i(TAG, values[0]+"");


        /*if(values[0] >= -5 && values[0] < 5){

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

        }*/
        //经过验证好像不是很准

    }

    public void showSpeed(){
        TextViewP.setText("Angular rate：\nXvel: " + (float) Math.toDegrees(anglespeedValues[0]) + "\nYvel: " +(float) Math.toDegrees(anglespeedValues[1]) + "\nZvel: " + (float) Math.toDegrees(anglespeedValues[2]));
//        if (state == 1) {
//            save2file(Long.toString(System.currentTimeMillis()) + "\t" +anglespeedValues[0]+ "\t" +anglespeedValues[1]+ "\t" +anglespeedValues[2]
//                    +"\t" + values[0] + "\t" +values[1] + "\t" + values[2]
//                    +"\t" + state_corrected_vel.get(0, 0)
//                    +"\t"+ state_corrected_vel.get(1, 0)
//                    +"\t"+ state_corrected_vel.get(2, 0)
//                    +"\t"+ state_corrected_vel.get(3, 0)
//                    +"\t"+ state_corrected_vel.get(4, 0)
//                    +"\t"+ state_corrected_vel.get(5, 0)+"\n");
//        }
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
            RandomAccessFile randomFile = new RandomAccessFile("/sdcard/"+fileName, "rws");
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
