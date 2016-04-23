package application;
 
import java.io.PrintWriter;
import java.net.Socket;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;



 
public class CameraCalib extends RoboticsAPIApplication {
    //private Controller kuka_Sunrise_Cabinet;
	//private Controller controller;
    private LBR lbr;
    double[] homePose = new double[] {Math.toRadians(-90),  Math.toRadians(90), 0, Math.toRadians(0), Math.toRadians(0), Math.toRadians(0), 0};
boolean firstRun=true;
boolean keyOK = false;
int poseNumber=1;
// Tool Data 
 
private Tool toolAttachedToLBR;
    private LoadData loadData;
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 0 };
    double startTime=0;
    public void initialize() {
        lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        loadData = new LoadData();
        toolAttachedToLBR = new Tool("Tool", loadData);
        toolAttachedToLBR.attachTo(lbr.getFlange());

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(translationOfTool[0], translationOfTool[1],translationOfTool[2]);
        ObjectFrame aTransformation = toolAttachedToLBR.addChildFrame(toolFrame+ "(TCP)", trans);
        toolAttachedToLBR.setDefaultMotionFrame(aTransformation);


        
        IUserKeyBar calibrationBar = getApplicationUI().createUserKeyBar("CCM");
        IUserKeyListener keyListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if(event == UserKeyEvent.KeyDown){
					startTime = System.currentTimeMillis(); // awake from sleep
					if(firstRun){firstRun=false;keyOK=true;return;}
					 Transformation calilbTrans = lbr.getCurrentCartesianPosition(toolAttachedToLBR.getDefaultMotionFrame()).getTransformationFromParent();
				    // Socket client
					 try {
				         Socket skt = new Socket("172.31.1.1", 5559);
				       
				         PrintWriter outCalib = new PrintWriter(skt.getOutputStream(), true);
				     String data = String.format( "%02d",  poseNumber)+", " +String.format( "%.2f",  calilbTrans.getX() )+ ", " +String.format( "%.2f",  calilbTrans.getY() )+ ", "  + String.format( "%.2f",  calilbTrans.getZ() )+
			        		 ", "  + String.format( "%.2f",  Math.toDegrees(calilbTrans.getAlphaRad()))+
			        		 ", "  + String.format( "%.2f",  Math.toDegrees(calilbTrans.getBetaRad()))+
			        		 ", "  + String.format( "%.2f",  Math.toDegrees(calilbTrans.getGammaRad()));
				     
				         System.out.println("Sent: " +data);
				         outCalib.println(data);
				         outCalib.close();
				         poseNumber++;
				         if(poseNumber>99)poseNumber=99; // limit
				           }
				      catch(Exception e) {
				         System.out.println("Whoops! It didn't work ! Check the connection to server");
				      }
					 
					 
					key.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Green, UserKeyLEDSize.Normal);
					}
					else if(event == UserKeyEvent.KeyUp){
						key.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
					}
			}
        	};
        	
        IUserKey calibKey = calibrationBar.addUserKey(0, keyListener, true);
        calibKey.setEnabled(true);
        calibKey.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
        calibrationBar.publish();
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is
     * collision free.
     */
    private void moveToInitialPosition()
    {
        toolAttachedToLBR.move(ptp(homePose).setJointVelocityRel(0.8));

         }
     // Main Loop    
    public void run() {
//back to home
     //   moveToInitialPosition();
        getLogger().info("Calibration Program V1.0");
        getLogger().info("---------------------------------");
        DelayMs(500);
        System.out.println("[1] Make sure KUKA is connected to log server 172.31.1.1:5559.");
        DelayMs(1000);
        System.out.println("[2] Make sure the camera is attached and running.");
        DelayMs(1000);
        System.out.println("[3] Move the robot in jog mode manually and press CCM.");
        DelayMs(1000);
        System.out.println("[4] Don't forget to capture a chessboard image at each pose.");
        DelayMs(1000);
        System.out.println("[5] Now, enable CCM switch from left panel and press it.");
        while(!keyOK);
        System.out.println("OK. Let's do it !");
        getLogger().info("---------------------------------");
        try{
        	 startTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - startTime) < 600000) {
            }//while
        }
        catch (Exception e){ e.printStackTrace();} 
        finally{
            System.out.println("OK, See You!");
        }
     return;    }
     /**
     * Auto-generated method stub. Do not modify the contents of this method.
     */
    public static void main(String[] args) {
        CameraCalib app = new CameraCalib();
        app.runApplication();   
    }
     /*****************************************************************************************************/
    public void DelayMs(double delay_time){
        long startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < delay_time);
    }

}