#include "socket.h"
#include "encoder.h"
#include "pwm.h"
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int speed=0;

int target = 0;
int target2 = 0;
int target3 = 0;
int target4 = 0;

// socket::socket()
// {
//     Serial.begin(baud);
// }
void socket::setMotorSpeed()
{
    Serial.print("Speed Settings: ");
    Serial.println(currentSpeedSettings);
}
void socket::setCurrentSpeed(speedSettings newSpeedSettings)
{
    currentSpeedSettings = newSpeedSettings;
    Serial.printf("currentSpeedSetting:= %s\n",currentSpeedSettings == SLOW ? "SLOW": currentSpeedSettings == NORMAL ? "NORMAL": currentSpeedSettings == FAST ? "FAST":"PROBLEM");
    
}

void socket::connectWifi()
{
    Serial.println("Connecting to ");
    Serial.println(ssid);

    // Connect to your wifi
    /**
     * ket noi toi wifi 
     */
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int cnt=0;
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(300);
      Serial.printf(">>");
      cnt++;
      if(cnt==30)
        esp_restart();
    }
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        /**
         * neu ma khong ket noi dc wifi in ra failed
         * va dung chuong trinh ngay tai do 
        */
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void socket::turnLeft()
{
    
 target=-speed;
  target2=speed;
  target3=speed;
  target4=-speed;

}
void socket::turnRight()
{
 target=speed;
  target2=-speed;
  target3=-speed;
  target4=speed;
}
void socket::moveForward()
{
  //*speed=500;
  target=-speed;
  target2=-speed;
  target3=-speed;
  target4=-speed;
}
void socket::moveBackward()
{
  //*speed=-500;
 

   target=speed;
  target2=speed;
  target3=speed;
  target4=speed;

}
void socket::stop()
{
  target=0;
  target2=0;
  target3=0;
  target4=0;
}
void socket::TienTrai()
{
   target=-speed;
  target2=0;
  target3=0;
  target4=-speed;
}
void socket::TienPhai()
{
 

  target=0;
  target2=-speed;
  target3=-speed;
  target4=0;
}
void socket::LuiTrai()
{
 
  target=speed;
  target2=0;
  target3=0;
  target4=speed;

  
}
void socket::LuiPhai()
{
   

  target=0;
  target2=speed;
  target3=speed;
  target4=0;
  
}
void socket::rolate_right()
{
   target=-speed;
 target2=-speed;
 target3=speed;
 target4=speed;
}
void socket::rolate_left()
{
 

  target=speed;
 target2=speed;
 target3=-speed;
 target4=-speed;
}

void socket::sendCarCommand(const char *command)
{
    // command could be either "left", "right", "forward" or "reverse" or "stop"
  // or speed settingg "slow-speed", "normal-speed", or "fast-speed"
  /**
   * neu commnad nhan duoc la tu left thi xe se quay trai
  */
  if (strcmp(command, "left") == 0)
  {
    _soc.turnLeft();
  }
  /**
   * neu command nhan duoc la tu right thi xe se quay phai 
  */
  else if (strcmp(command, "right") == 0)
  {
    _soc.turnRight();
  }
  /**
   * neu command nhan duoc la tu up thi xe se tien len phia truoc 
  */
  else if (strcmp(command, "up") == 0)
  {
    _soc.moveForward();
  }
  /**
   * neu command nhan duoc la tu down thi xe se di lui 
  */
  else if (strcmp(command, "down") == 0)
  {
    _soc.moveBackward();
  }
  /**
   * neu command nhan duoc la tu stop thi robot se dung lai 
  */
  else if (strcmp(command, "stop") == 0)
  {
    _soc.stop();
  }
  /**
   * neu command nhan duoc la slow-speed thi co nghia la toc do cua xe
   * se la muc SLOW 
  */
  else if (strcmp(command, "slow-speed") == 0)
  {
    _soc.setCurrentSpeed(speedSettings::SLOW);
    speed=600;
  }
  /**
   * neu command nhan duoc la normal-speed thi toc do xe se la NORMAL
  */
  else if (strcmp(command, "normal-speed") == 0)
  {
    _soc.setCurrentSpeed(speedSettings::NORMAL);
    speed=1000;
  }
  /**
   * neu command nhan duoc la fast-speed thi xe se chay voi toc do cao 
  */
  else if (strcmp(command, "fast-speed") == 0)
  {
    _soc.setCurrentSpeed(speedSettings::FAST);
    speed=1500;
  }
  else if((strcmp(command,"up-left") == 0))
  {
    // lui phai 
    //_soc.setCurrentSpeed(speedSettings::MOVE_LEFT);
    _soc.TienTrai();
  }
  else if((strcmp(command,"up-right") == 0))
  {
    // tien trai 
    //_soc.setCurrentSpeed(speedSettings::MOVE_RIGHT);
    _soc.TienPhai();


  }
  else if((strcmp(command,"down-left") == 0))
  {

    //_soc.setCurrentSpeed(speedSettings::BACK_LEFT);
    _soc.LuiTrai();
  }
  else if((strcmp(command,"down-right") == 0))
  {
    // tien phai 
    //_soc.setCurrentSpeed(speedSettings::BACK_RIGHT);
    _soc.LuiPhai();
  }
   else if((strcmp(command,"rotate-left") == 0))
  {
    //Serial.println("rolate left");
    //_soc.readyCar();
    //_soc.setCurrentSpeed(speedSettings::BACK_LEFT);
    _soc.rolate_left();
  }
  else if((strcmp(command,"rotate-right") == 0))
  {
    // tien phai 
    //_soc.setCurrentSpeed(speedSettings::BACK_RIGHT);
    _soc.rolate_right();
  }

}

void socket::readyCar()
{
    pinMode(2,OUTPUT);
    digitalWrite(2,1);
}

String socket::indexPageProcessor(const String &var)
{
  String status = "";

  if (var == "SPEED_SLOW_STATUS")
  {
   
    if (_soc.getCurrentSpeed() == speedSettings::SLOW)
    {
      status = "checked";
    }
  }
  
  else if (var == "SPEED_NORMAL_STATUS")
  {
    
    if (_soc.getCurrentSpeed() == speedSettings::NORMAL)
    {
      status = "checked";
    }
  }
 
  else if (var == "SPEED_FAST_STATUS")
  {
    
    if (_soc.getCurrentSpeed() == speedSettings::FAST)
    {
      status = "checked";
    }
  }
  // else if(var == "MOVE_LEFT")
  // {
  //   if (_soc.getCurrentSpeed() == speedSettings::MOVE_LEFT)
  //   {
  //     status = "checked";
  //   }
  // }
  // else if(var == "MOVE_RIGHT")
  // {
  //   if (_soc.getCurrentSpeed() == speedSettings::MOVE_RIGHT)
  //   {
  //     status = "checked";
  //   }
  // }
  // else if(var == "BACK_LEFT")
  // {
  //   if (_soc.getCurrentSpeed() == speedSettings::BACK_LEFT)
  //   {
  //     status = "checked";
  //   }
  // }
  // else if(var == "BACK_RIGHT")
  // {
  //   if (_soc.getCurrentSpeed() == speedSettings::BACK_RIGHT)
  //   {
  //     status = "checked";
  //   }
  // }
  
  return status;
}

void socket::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  
  switch (type)
  {
    
  case WS_EVT_CONNECT:
  {
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    
  }
 
  case WS_EVT_DISCONNECT:
  {
    Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
  }
  
  case WS_EVT_DATA:
  {

    
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    
    if (info->final && info->index == 0 && info->len == len)
    {

      
      if (info->opcode == WS_TEXT)
      {
        
        data[len] = 0;
        
        char *command = (char *)data;
        _soc.sendCarCommand(command);
      }
    }
  }

  case WS_EVT_PONG:
  {
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
  }

  case WS_EVT_ERROR:
  {
    
  }
  }
}

void socket::notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

void socket::initSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
}

void socket::initWebsocket()
{
    ws.onEvent(socket::onWsEvent);

    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Requesting index page...");
        request->send(SPIFFS, "/index.html", "text/html", false, socket::indexPageProcessor);
    });

    server.on("/css/entireframework.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/css/entireframework.min.css", "text/css"); });

    server.on("/css/custom.css", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/css/custom.css", "text/css"); });

    server.on("/js/custom.js", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/js/custom.js", "text/javascript"); });

    server.onNotFound(socket::notFound);


    server.begin();
}


socket _soc;