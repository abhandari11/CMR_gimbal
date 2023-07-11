// Read Digital Relay value from Ardupilot


const int relay_pin = 25;
int _relay_threshold = 10;
bool _relay_state = false;
int _state_counter = 0;


void setup() {
  // put your setup code here, to run once:
  // initialize serial communication
  Serial.begin(38400);
  while (!Serial); 
  
  // setting the digital relay switch
  // from the flight computer
  pinMode(relay_pin, INPUT);
  Serial.println("-----------------------");
  Serial.print("The relay is : ");
  Serial.print(digitalRead(relay_pin));
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int relay_cmd = digitalRead(relay_pin);
  Serial.println(relay_cmd);

  if (!_relay_state) {
    if (relay_cmd == 1) {
      _state_counter++;
      }
      
    else {
      _state_counter = 0;
      }
    }
    
  else if (_relay_state) {
    if (relay_cmd == 0) {
      _state_counter++;
      }
    else {
      _state_counter = 0;
      }
    }
  Serial.print("State counter >> ");
  Serial.println(_state_counter);
  
  if (_state_counter > _relay_threshold) {
    _state_counter = 0;
    _relay_state = !_relay_state;
    }
  
  if (_relay_state) {
    Serial.println("RUNNING...");
    }
    
  delay(200);
}
