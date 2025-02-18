const int buttonPin = 2;  // the number of the pushbutton pin


void setup() {
  // put your setup code here, to run once:
    // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    Serial.print("button pressed");
  }
  else {

  }
}
