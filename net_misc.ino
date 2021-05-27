
void print_ip_status(){
  Serial.print("\nWiFi connected !\n");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print("\n");
  Serial.print("MAC address: ");
  Serial.print(WiFi.macAddress());
  Serial.print("\n");
}
String ssid;
 String password;
void connect_wifi(){
 // Access Point of the infrastructure
 //const char* ssid = "HUAWEI-6EC2";
 //const char *password= "FGY9MLBL";
 //const char* ssid = "HUAWEI-553A";

   preferences.begin("credentials", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
          preferences.end();
 Serial.println("\nConnecting Wifi to ");
 Serial.println(ssid.c_str());

 Serial.print("Attempting to connect ");
 WiFi.begin(ssid.c_str(), password.c_str());
 while(WiFi.status() != WL_CONNECTED && !isEmit){
   if (button1.pressed) {
     isEmit = true;
      button1.pressed = false;
     }
       digitalWrite(ONBOARD_LED,HIGH);
   delay(500);
        digitalWrite(ONBOARD_LED,LOW);
          delay(500);
          Serial.print(ssid.c_str());
          Serial.print(password.c_str());
   Serial.print(".");
 }

 print_ip_status();
}
