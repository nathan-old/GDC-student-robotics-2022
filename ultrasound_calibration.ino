#define trigPin 12
#define echoPin 11
#define buttonPin 10

int buttonState = 0;

void setup() {
    Serial.begin (115200);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

void distance() {
    digitalWrite(LED_BUILTIN, HIGH);
    long duration, distance;
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2);
    Serial.println(distance);
    Serial.print(" ");
    // return distance;
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
        // run distance function
        distance();
    }
}

// use serial plotter to calculate values for distance measured then calculate the gradient as a convertion factor.
