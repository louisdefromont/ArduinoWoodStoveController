#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h> 
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80

float temperature = 0;

void handleRoot();              // function prototypes for HTTP handlers
void handleLED();
void handleNotFound();

void setup(void){
  Serial.begin(115200);         // Start the Serial communication to send messages to the computer
  delay(10);
  // Serial.println('\n');

  wifiMulti.addAP("Freebox-1D640E", "sutos2-colludium**-effigiat&*-edurare67");   // add Wi-Fi networks you want to connect to

  // Serial.println("Connecting ...");
  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(250);
    // Serial.print('.');
  }
  // Serial.println('\n');
  // Serial.print("Connected to ");
  // Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  // Serial.print("IP address:\t");
  // Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer

  // if (MDNS.begin("esp8266")) {              // Start the mDNS responder for esp8266.local
  //   Serial.println("mDNS responder started");
  // } else {
  //   Serial.println("Error setting up MDNS responder!");
  // }

  server.on("/", HTTP_GET, handleRoot);     // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/LED", HTTP_POST, handleLED);  // Call the 'handleLED' function when a POST request is made to URI "/LED"
  server.on("/api/temperature", HTTP_GET, getTemperature);
  server.on("/motor_control", HTTP_GET, handleMotorControl);
  server.on("/api/motor_direction", HTTP_POST, postMotorDirection);
  server.on("/api/motor_control", HTTP_POST, postMotorControl);
  server.on("/api/temperature_target", HTTP_POST, postTemperatureTarget);
  server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

  server.begin();                           // Actually start the server
  // Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();                    // Listen for HTTP requests from clients

  if (Serial.available() > 0) {
    String read = Serial.readString();
    temperature = read.toFloat();
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Temperature Display</title>";
  html += "<style>";
  html += "html, body { height: 100%; margin: 0; display: flex; align-items: center; justify-content: center; }";
  html += "#temperature-container { text-align: center; }";
  html += "#calibrate-btn { margin-top: 20px; }";
  html += "</style>";
  html += "<script>function refreshTemperature() {";
  html += "var xhttp = new XMLHttpRequest();";
  html += "xhttp.onreadystatechange = function() {";
  html += "if (this.readyState == 4 && this.status == 200) {";
  html += "document.getElementById('temperature').innerHTML = this.responseText;";
  html += "}";
  html += "};";
  html += "xhttp.open('GET', '/api/temperature', true);";
  html += "xhttp.send();";
  html += "}";
  html += "setInterval(refreshTemperature, 1000);"; // Refresh temperature every second
  html += "</script></head><body>";
  html += "<div id='temperature-container'>";
  html += "<p><span id='temperature' style='font-size: 35vw;font-family: sans-seriff;'>" + String(temperature) + "</span></p>";
  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleMotorControl() {
  String html = "<!DOCTYPE html><html><head><title>Motor Control</title>";
  html += "<style>";
  html += "html, body { height: 100%; margin: 0; display: flex; align-items: center; justify-content: center; }";
  html += "button { padding: 15px 30px; font-size: 18px; margin: 10px; }";
  html += ".button-container { display: flex; flex-direction: row; align-items: center; }";
  html += ".container { margin-bottom: 20px; display: flex; flex-direction: column; justify-content: center; }";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<div class='button-container'>";
  html += "<button onclick=\"sendMotorDirection(-1)\">Left</button>";
  html += "<button onclick=\"sendMotorTarget(-1)\">Stop</button>";
  html += "<button onclick=\"sendMotorDirection(1)\">Right</button>";
  html += "</div>";
  html += "<div class='button-container'>";
  html += "<button onclick=\"sendMotorTarget(0)\">0.0%</button>";
  html += "<button onclick=\"sendMotorTarget(5)\">12.5%</button>";
  html += "<button onclick=\"sendMotorTarget(10)\">25.0%</button>";
  html += "<div class='button-container'>";
  html += "</div>";
  html += "<button onclick=\"sendMotorTarget(15)\">37.7%</button>";
  html += "<button onclick=\"sendMotorTarget(20)\">50.0%</button>";
  html += "<button onclick=\"sendMotorTarget(25)\">62.5%</button>";
  html += "<div class='button-container'>";
  html += "</div>";
  html += "<button onclick=\"sendMotorTarget(30)\">75.0%</button>";
  html += "<button onclick=\"sendMotorTarget(35)\">87.5%</button>";
  html += "<button onclick=\"sendMotorTarget(40)\">100.0%</button>";
  html += "</div>";
  html += "<div class='button-container'>";
  html += "<input type='text' id='tempInput' placeholder='Enter temperature'>";
  html += "<button onclick=\"sendTemperatureTarget()\">Set Target Temperature</button>";
  html += "</div>";
  html += "</div>";
  html += "<script>";
  html += "function sendMotorDirection(value) {";
  html += "var xhttp = new XMLHttpRequest();";
  html += "xhttp.open('POST', '/api/motor_direction', true);";
  html += "xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');";
  html += "xhttp.send('direction=' + value);";
  html += "}";
  html += "function sendMotorTarget(value) {";
  html += "var xhttp = new XMLHttpRequest();";
  html += "xhttp.open('POST', '/api/motor_control', true);";
  html += "xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');";
  html += "xhttp.send('target=' + value);";
  html += "}";
  html += "function sendTemperatureTarget() {";
  html += "var temperature = document.getElementById('tempInput').value;";
  html += "var xhttp = new XMLHttpRequest();";
  html += "xhttp.open('POST', '/api/temperature_target', true);";
  html += "xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');";
  html += "xhttp.send('target=' + temperature);";
  html += "}";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

void handleLED() {                          // If a POST request is made to URI /LED
  server.sendHeader("Location","/");        // Add a header to respond with a new location for the browser to go to the home page again
  server.send(303);                         // Send it back to the browser with an HTTP status 303 (See Other) to redirect
}

void getTemperature() {
  server.send(200, "text/plain", String(temperature));
}

void postMotorDirection() {
  if (server.hasArg("direction")) {
    String directionArg = server.arg("direction");
    Serial.println("D" + directionArg);
  }

  server.send(200, "text/plain", ""); // Respond with an empty body to acknowledge the request
}

void postMotorControl() {
  if (server.hasArg("target")) {
    String targetArg = server.arg("target");
    Serial.println("M" + targetArg);
  }

  server.send(200, "text/plain", ""); // Respond with an empty body to acknowledge the request
}

void postTemperatureTarget() {
  if (server.hasArg("target")) {
    String targetArg = server.arg("target");
    Serial.println("T" + targetArg);
  }

  server.send(200, "text/plain", ""); // Respond with an empty body to acknowledge the request
}

void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}