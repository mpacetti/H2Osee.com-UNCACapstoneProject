package com.h2osee.android.mqttseeclient;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Vibrator;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;


public class ConfigActivity extends AppCompatActivity {


    TextView publishedMsg;
    TextView statusMsgTv;

    EditText EtBrokerAddress;
    EditText EtBrokerPort;

    MqttHelper mqHelp;
    MqttAndroidClient client;
    MqttConnectOptions options = new MqttConnectOptions();

    String brokerAddress = "mqttbroker.h2osee.com";
    String brokerPort = "1883";
    String brokerUri = "tcp://" + brokerAddress + ":" + brokerPort;
    String clientId;

    Button statusBtn;

    Vibrator vib;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_config);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        getSupportActionBar().setTitle("MQTTsee Broker CONFIG");
        vib = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);


        final Button dashBtn = (Button) findViewById(R.id.dashboardBtn);
        final Button monBtn = (Button) findViewById(R.id.monitorBtn);

        statusBtn = (Button) findViewById(R.id.statusBtn);
        statusMsgTv = (TextView) findViewById(R.id.statusMsgTv);
        publishedMsg = (TextView) findViewById(R.id.statusMsgTv);

        EtBrokerAddress = (EditText)findViewById(R.id.config_broker_address);
        EtBrokerPort = (EditText)findViewById(R.id.config_broker_port);

        // begin disconnected state
        statusMsgTv.setText(R.string.DISC);
        statusMsgTv.setTextColor(getApplication().getResources().getColor(R.color.colorRed));


        // dashboard button click listener
        dashBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myDashIntent = new Intent(ConfigActivity.this, DashboardActivity.class);
                ConfigActivity.this.startActivity(myDashIntent);
            }
        });

        // monitor button click listener
        monBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myMonIntent = new Intent(ConfigActivity.this, MonitorActivity.class);
                ConfigActivity.this.startActivity(myMonIntent);
            }
        });


    } // end of onCreate()



    private void startMqtt() {


        mqHelp = new MqttHelper(getApplicationContext());

        mqHelp.setCallback(new MqttCallbackExtended() {

            @Override
            public void connectComplete(boolean b, String s) {

            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                Log.w("Debug",mqttMessage.toString());

                statusMsgTv.setText(topic + "  ->  " + mqttMessage.toString() + "\n" + statusMsgTv.getText());

                //statusMsgTv.append(topic + "  ->  ");
                //statusMsgTv.append(mqttMessage.toString() + "\n");

            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }




    public void connect(View v) {

        String brokerAdd = EtBrokerAddress.getText().toString();
        String brokerP = EtBrokerPort.getText().toString();

        if (EtBrokerAddress.getText().toString().isEmpty()) {

            //brokerUri = "tcp://" + brokerAdd + ":" + brokerPort;
            Log.d("TAG-empty-broker", brokerUri);
            //brokerUri = "tcp://" + EtBrokerAddress.getText().toString() + ":" + brokerPort;

        } else {

            brokerUri = "tcp://" + brokerAdd + ":" + brokerPort;
            Log.d("TAG-typedin-broker", brokerUri);
        }


        if (EtBrokerPort.getText().toString().isEmpty()) {
            Log.d("TAG-empty-port", EtBrokerPort.getText().toString());
            //brokerUri = "tcp://" + brokerAddress + ":" + EtBrokerPort.getText().toString();
        }


        clientId = MqttClient.generateClientId();
        client = new MqttAndroidClient(this.getApplicationContext(), brokerUri, clientId);

        if(!client.isConnected()) {
            Log.d("TASG-->xxx", brokerUri);
            try {

                IMqttToken token = client.connect(options);
                token.setActionCallback(new IMqttActionListener() {
                    @Override
                    public void onSuccess(IMqttToken asyncActionToken) {
                        Toast.makeText(getApplicationContext(), "connected to broker!", Toast.LENGTH_SHORT).show();
                        statusMsgTv.setText("CONNECTED to " + brokerAddress + ":" + brokerPort + "\n");
                        vib.vibrate(400);
                        statusBtn.setText(R.string.CONN);
                        statusBtn.setBackground(getResources().getDrawable(R.drawable.button_bg_green));
                    }

                    @Override
                    public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                        Toast.makeText(getApplicationContext(), "could not connect to broker...", Toast.LENGTH_SHORT).show();
                    }
                });

            } catch (MqttException e) {

                e.printStackTrace();
            }
        }
    }


    public void disconnect(View v) {

        if (client.isConnected()) {

            try {

                IMqttToken token = client.disconnect();
                token.setActionCallback(new IMqttActionListener() {

                    @Override
                    public void onSuccess(IMqttToken asyncActionToken) {

                        Toast.makeText(getApplicationContext(), "disconnected from broker!", Toast.LENGTH_LONG).show();
                        statusMsgTv.setText("DISCONNECTED from " + brokerAddress + ":" + brokerPort + "\n");
                        vib.vibrate(400);
                        statusBtn.setText(R.string.DISC);
                        statusBtn.setBackground(getResources().getDrawable(R.drawable.button_bg_red));
                        EtBrokerAddress.setText("");
                    }

                    @Override
                    public void onFailure(IMqttToken asyncActionToken, Throwable exception) {

                        Toast.makeText(getApplicationContext(), "could not disconnect from broker...", Toast.LENGTH_LONG).show();
                    }
                });

            } catch (MqttException e) {

                e.printStackTrace();
            }
        }
    }


    public void subscribe(View v) {

        if (client.isConnected()) {

            startMqtt();

        }

    }

}
