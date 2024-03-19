void setup(){
    Serial.begin(115200);
}

float distance = 0;
void loop(){
    distance = (float)random(0, 50);
    Serial.println(distance);
    delay(100);
}