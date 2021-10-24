package pl.edu.uj.aml_compas;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import java.util.Timer;
import java.util.TimerTask;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;
import android.widget.ImageView;
import android.widget.TextView;

public class Compas extends AppCompatActivity implements SensorEventListener {

    public final static String TAG = "Rotation";

    // Refresh rate for updating fused orientation
    public static  int REFRESH_INTERVAL = 30;

    //filtering gyroscope
    public static  float FILTER_COEFFICIENT = 0.98f;

    private static final float NS2S = 1.0f / 1000000000.0f;
    public static final float OMEGA_MAGNITUDE_EPSILON = 0.000000001f;

    // define the display assembly compass picture
    private ImageView image;

    // record the compass picture angle turned
    private float currentDegree = 0f;

    // device sensor manager
    private SensorManager mSensorManager;

    private Sensor mSensorAcc;
    private Sensor mSensorMgn;
    private Sensor mSensorGyro;

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    // real rotation matrix, dependant on screen orientation
    private float[] realRotationMatrix = new float[9];

    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // magnetic field vector
    private float[] magnet = new float[3];

    // accelerometer vector
    private float[] accel = new float[3];

    //data for sensor fusion
    private float gyroTimestamp;
    private Timer fuseTimer;
    private boolean initState = true;

    // fused sensors orientation
    private float[] fusedOrientation = new float[3];;

    // orientation provided by gyroscope
    private float[] gyroOrientation = new float[3];
    private float[] gyroMatrix =  {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,0.0f, 0.0f, 0.0f,1.0f};

    //coordinate rotation
    int COORDINATES_ROTATION_X;
    int COORDINATES_ROTATION_Y;

    TextView tvHeading;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_compas);

        // our compass image
        image = (ImageView) findViewById(R.id.imageViewCompass);

        // TextView that will tell the user what degree is he heading
        tvHeading = (TextView) findViewById(R.id.tvHeading);

        // initialize your android device sensor capabilities
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensorAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensorMgn = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        Display display = getWindowManager().getDefaultDisplay();
        int deviceRot = display.getRotation();

        //Decide how to remap coordinate system
        switch (deviceRot)
        {
            // natural
            case Surface.ROTATION_0:
                COORDINATES_ROTATION_X = SensorManager.AXIS_X;
                COORDINATES_ROTATION_Y = SensorManager.AXIS_Y;
                Log.d(TAG, "Natural position.");
                break;
            // rotated left )
            case Surface.ROTATION_90:
                COORDINATES_ROTATION_X = SensorManager.AXIS_Y;
                COORDINATES_ROTATION_Y = SensorManager.AXIS_MINUS_X;
                Log.d(TAG, "Rotation 90 degrees.");
                break;
            // upside down
            case Surface.ROTATION_180:
                COORDINATES_ROTATION_X = SensorManager.AXIS_MINUS_X;
                COORDINATES_ROTATION_Y =SensorManager.AXIS_MINUS_Y;
                Log.d(TAG, "Rotation 180 degrees.");
                break;
            // rotated right
            case Surface.ROTATION_270:
                COORDINATES_ROTATION_X = SensorManager.AXIS_MINUS_Y;
                COORDINATES_ROTATION_Y = SensorManager.AXIS_X;
                Log.d(TAG, "Rotation 270 degrees.");
                break;

            default:  break;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();

        // for the system's orientation sensor registered listeners
        if(mSensorAcc != null)
            mSensorManager.registerListener(this, mSensorAcc, SensorManager.SENSOR_DELAY_FASTEST);
        if(mSensorMgn != null )
            mSensorManager.registerListener(this, mSensorMgn, SensorManager.SENSOR_DELAY_FASTEST);
        if(mSensorGyro != null)
            mSensorManager.registerListener(this, mSensorGyro, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPause() {
        super.onPause();

        // to stop the listener and save battery
        mSensorManager.unregisterListener(this);
    }

    @Override
    protected void onStop() {
        // TODO Auto-generated method stub
        super.onStop();

        fuseTimer.cancel();
        fuseTimer.purge();
    }

    @Override
    protected void onStart() {
        // TODO Auto-generated method stub
        super.onStart();
        fuseTimer = new Timer();
        fuseTimer.scheduleAtFixedRate(new fuseSensorTask(),
                1000, REFRESH_INTERVAL);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final float alpha = 0.66f;
        synchronized(gyroOrientation){
            switch(event.sensor.getType()) {
                case Sensor.TYPE_MAGNETIC_FIELD:
                    //apply low-pass filter
                    magnet[0] = alpha*magnet[0] + (1-alpha)*event.values[0];
                    magnet[1] = alpha*magnet[1] + (1-alpha)*event.values[1];
                    magnet[2] = alpha*magnet[2] + (1-alpha)*event.values[2];
                    break;
                case Sensor.TYPE_ACCELEROMETER:
                    accel[0] = alpha*accel[0] + (1-alpha)*event.values[0];
                    accel[1] = alpha*accel[1] + (1-alpha)*event.values[1];
                    accel[2] = alpha*accel[2] + (1-alpha)*event.values[2];
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    // copy the new gyro values into the gyro array
                    // convert the raw gyro data into a rotation vector
                    if (accMagOrientation == null)
                        return;

                    // initialisation of the gyroscope based rotation matrix
                    if(initState) {
                        float[] initMatrix = new float[9];
                        initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
                        float[] test = new float[3];
                        SensorManager.getOrientation(initMatrix, test);
                        gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
                        initState=false;
                    }

                    float[] deltaVector = new float[4];
                    float[] gyro = new float[3];
                    if(gyroTimestamp != 0) {
                        final float dT = (event.timestamp - gyroTimestamp) * NS2S;
                        //Since we remapCoordinates, the z and y axis are swaped we need to swap the gyro readings
                        System.arraycopy(event.values, 0, gyro, 0, 3);
                        getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
                    }

                    // measurement done, save current time for next interval
                    gyroTimestamp = event.timestamp;

                    // convert rotation vector into rotation matrix
                    float[] deltaMatrix = new float[9];
                    SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);
                    // apply the new rotation interval on the gyroscope based rotation matrix
                    gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);
                    SensorManager.getOrientation(gyroMatrix, gyroOrientation);

                    break;
                default:
                    return;
            }
            calculateAccMagOrientatoin();
        }
        animateCompas();
    }



    public void animateCompas(){
        synchronized(gyroOrientation){
            float [] fusedRotation =  getRotationMatrixFromOrientation(fusedOrientation);
            SensorManager.remapCoordinateSystem(fusedRotation,
                    COORDINATES_ROTATION_X, COORDINATES_ROTATION_Y,
                    realRotationMatrix);
        }
        float remapedOrientation[] = new float[3];
        SensorManager.getOrientation(realRotationMatrix, remapedOrientation);
        double degree =  Math.toDegrees(remapedOrientation[0]);
        if (degree < 0.0f) {
            degree += 360.0f;
        }

        degree = (Math.round(degree*100.0))/100.0;

        tvHeading.setText("Heading: " + Double.toString(degree) + " degrees");

        // create a rotation animation (reverse turn degree degrees)
        RotateAnimation ra = new RotateAnimation(
                currentDegree,
                (float) -degree,
                Animation.RELATIVE_TO_SELF, 0.5f,
                Animation.RELATIVE_TO_SELF,
                0.5f);

        // how long the animation will take place
        ra.setDuration(210);

        // set the animation after the end of the reservation status
        ra.setFillAfter(true);

        // Start the animation
        image.startAnimation(ra);
        currentDegree = (float) -degree;
    }

    private void calculateAccMagOrientatoin(){
        if (SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)){
            if(accMagOrientation == null) accMagOrientation = new float[3];

            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }



    class fuseSensorTask extends TimerTask {
        public void run() {
            synchronized(gyroOrientation){
                if(accMagOrientation != null){
                    float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
                    /*
                     * Fix for 179ÄĹźË <--> -179ÄĹźË transition problem:
                     * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
                     * If so, add 360ÄĹźË (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360ÄĹźË from the result
                     * if it is greater than 180ÄĹźË. This stabilizes the output in positive-to-negative-transition cases.
                     */
                    // azimuth
                    if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
                        fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]);
                        fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
                    }
                    else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
                        fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
                        fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
                    }
                    else {
                        fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
                    }

                    // pitch
                    if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
                        fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]);
                        fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
                    }
                    else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
                        fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
                        fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
                    }
                    else {
                        fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
                    }

                    // roll
                    if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
                        fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]);
                        fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
                    }
                    else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
                        fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
                        fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
                    }
                    else {
                        fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
                    }


                    // overwrite gyro matrix and orientation with fused orientation
                    // to comensate gyro drift
                    gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
                    System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

                }
            }
        }


    };



    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float) Math.sin(o[1]);
        float cosX = (float) Math.cos(o[1]);
        float sinY = (float) Math.sin(o[2]);
        float cosY = (float) Math.cos(o[2]);
        float sinZ = (float) Math.sin(o[0]);
        float cosZ = (float) Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    // This function is borrowed from the Android reference
    // at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
    // It calculates a rotation vector from the gyroscope angular speed values.
    private void getRotationVectorFromGyro(float[] gyroValues,
                                           float[] deltaRotationVector,
                                           float timeFactor)
    {
        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude = (float) Math.sqrt(gyroValues[0] * gyroValues[0] +
                gyroValues[1] * gyroValues[1] +
                gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        if(omegaMagnitude > OMEGA_MAGNITUDE_EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not in use
    }
}