/*
# H2Osee.com
#
# mqtt.js for barometer page
#
# MARK PACETTI
#
# client application to connect to the MQTT broker and subscribe to topics and update UI
#
# Oct 1, 2019
#
# =======================================================================================-->*/
var mqtt;
var reconnectTimeout = 2500;
var client_name_prefix = "h2osee_websock_client_";

host = '34.73.67.18';
port = 8083;
topic = '#';
useTLS = false;
username = null;
password = null;
cleansession = true;

function MQTTconnect() {

    if (typeof path == "undefined") {
        var path = '/';
    }
    mqtt = new Paho.MQTT.Client(
        host,
        port,
        path,
        client_name_prefix + parseInt(Math.random() * 100, 10)
    );

    var options = {
        useSSL: false,
        timeout: 3,
        cleanSession: cleansession,
        onSuccess: onConnect,
        onFailure: function (message) {
            $('#status').val("connection failed: " + message.errorMessage + "retrying...");
            setTimeout(MQTTconnect, reconnectTimeout);
        }
    }

    //  setup callback for when connection is lost
    mqtt.onConnectionLost = onConnectionLost;
    //  setup callback for when a new MQTT message arrives
    mqtt.onMessageArrived = onMessageArrived;
    //  connect with option settings configured
    mqtt.connect(options);
}

function onConnect() {

    $('#status').val('CONNECTED to ' + host + ':' + port);
    // connection succeeded; now subscribe to all topics
    mqtt.subscribe(topic, {qos: 2});
    // update topic
    $('#topic').val(topic);
}

function onConnectionLost(message) {

    setTimeout(MQTTconnect, reconnectTimeout);
    $('#status').val("connection lost: " + message.errorMessage + ". Reconnecting");

}

function onMessageArrived(message) {

    var topic = message.destinationName;
    var payload = message.payloadString;

    // this is wheat is doing the dynamic updating of gauges from messages received
    $('#dashboard_gauge').prepend('<li>' + topic + ' = ' + payload + '</li>');
    // update the DOM with new MQTT values with jQuery
    updateSensorGauges(topic, payload);
}

function updateSensorGauges(msgTopic, msgPayload) {

    var topicArray = msgTopic.split("/");
    var sensor = topicArray[4];

    var date = new Date();
    //var time = date.toLocavarimeString().replace(/([\d]+:[\d]{2})(:[\d]{2})(.*)/, "$1$3")
    var time = date.toLocaleTimeString('en-US');
    $('.updated_time').text(time);

    switch(sensor) {
        case "Altitude":
            $('#alt').text(msgPayload);
            break;
        case "Latitude":
            $('#lat').text(msgPayload);
            break;
        case "Longitude":
            $('#lng').text(msgPayload);
            break;
        case "Air Temperature @ 2ft":
            $('#air_temp').text(msgPayload);
            break;
        case "Barometric Pressure":
            $('#baro').text(msgPayload);
            break;
        case "Humidity":
            $('#humid').text(msgPayload);
            break;
        case "Water Temperature @ 0ft":
            $('#water0').text(msgPayload);
            break;
        case "Water Temperature @ 2ft":
            $('#water2').text(msgPayload);
            break;
        case "UV Index":
            $('#uv').text(msgPayload);
            break;
        case "Turbidity NTU":
            $('#turbid').text(msgPayload);
            break;
        case "TDS":
            $('#tds').text(msgPayload);
            break;
        case "pH":
            $('#ph').text(msgPayload);
            break;
        default:
            // do nothing
    }
}

<!-- update real-time date/time inside jumbotron right column -->
function getClock() {

    var textday = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
    var textmonth = ["January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"];

    var d = new Date();
    var nday = d.getDay(), nmonth = d.getMonth(), ndate = d.getDate(), nyear = d.getFullYear();
    var nhour = d.getHours(), nmin = d.getMinutes(), nsec = d.getSeconds(), ap;

    if (nhour == 0) {
        ap = " AM";
        nhour = 12;
    } else if (nhour < 12) {
        ap = " AM";
    } else if (nhour == 12) {
        ap = " PM";
    } else if (nhour > 12) {
        ap = " PM";
        nhour -= 12;
    }

    if (nmin <= 9) nmin = "0" + nmin;
    if (nsec <= 9) nsec = "0" + nsec;

    let clock_text = "" + textday[nday] + ", " + textmonth[nmonth] + " " + ndate + ", " + nyear + " " + nhour + ":" + nmin + ":" + nsec + ap + "";
    $('#clockbox').text(clock_text)
}


$(document).ready(function() {
    MQTTconnect();
});

