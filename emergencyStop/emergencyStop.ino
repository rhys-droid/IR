const int buttonPin = 8;
int buttonState;

void setup(){
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);

}

void loop (){
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
}