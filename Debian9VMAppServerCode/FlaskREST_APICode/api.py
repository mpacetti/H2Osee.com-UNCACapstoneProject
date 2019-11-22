from flask import Flask, render_template, request, jsonify
import mysql.connector
from mysql.connector import Error
import json
import collections
import datetime

# gcp cloud IP address
# dbserver = "35.245.229.164"
# from anywhere else using proxy IP address
dbserver = "127.0.0.1"
dbname = "h2oseedb"
dbuser = "dbuser"
dbpassword = "DBpassword2019!"
dbtable = "sample"
timezone = "SET TIME_ZONE = 'US/Eastern'"


app = Flask(__name__)
app.config["DEBUG"] = True


@app.route('/', methods=['GET'])
def home():
    return 'Hello, world!!!'


@app.route('/graph', methods=['GET'])
def index():
    return "<h1>{sensor} graph</h1>"


@app.route("/<string:sensor>")
def getSensorVals(sensor):
    return f"Hello, {sensor}"

    
@app.route("/data.json")
def data():

    connection = mysql.connector.connect(host=dbserver, database=dbname, user=dbuser,password=dbpassword)
    cursor = connection.cursor(dictionary=True, buffered=True)
    cursor.execute(timezone)

    # cursor.execute("SELECT dbtimestamp, reading FROM sample WHERE sensor = 'humidity' and reading<>''")
    cursor.execute("SELECT date_format(dbtimestamp,'%Y-%m-%d %H') as date_hour, FORMAT(AVG(reading),2) as reading FROM sample WHERE sensor='humidity' AND reading<>'' GROUP BY date_hour")

    rows = cursor.fetchall()

    dts = []
    readings = []
    for row in rows:
        dt = row["date_hour"]
        reading = float(row["reading"])
        dts.append(dt)
        readings.append(reading)

    print(json.dumps(dts))
    print(json.dumps(readings))

    return json.dumps(readings)

app.run()
