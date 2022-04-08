int  settime;
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
float potValue[3] = {343.14, 330.0102, 343} ;
float x, y, z, S[3], v[3]; // S_x,S_y,v_x,v_y
float avg[3][100], a[3], g = 9.8, dt = 0;
long last_time = millis();
void setup() { 
  Serial.begin(9600);
  delay(100); 
} 

void avg_func(float aint[]){
  float avg_a[3] = {0, 0, 0};
  int maxrun = 99;
  if (settime < 100){ maxrun = settime;}
  
  for (int i = 0; i<=2; i++){
    avg[i][settime % (maxrun+1)] = aint[i]; 
    
      for (int ii = 0; ii<=maxrun; ii++){ avg_a[i] += avg[i][ii]; }
      a[i] = avg_a[i]/ (maxrun+1);
  }
}

void error(){
  settime = 0;
  for (int i = 0; i<=100; i++){
    x = (analogRead(xpin) - potValue[0])/69.5 * g;
    y = (analogRead(ypin) - potValue[1])/68.5 * g;
    z = (analogRead(zpin) - potValue[2])/67 * g;
    float temp[3] = {x,y,z};
    avg_func(temp);

    /*
    Serial.print("raw:  ");
    for (int i = 0; i<2; i++){
      Serial.print(temp[i], 2);
      Serial.print('\t');
    } 
  
    Serial.print("acc:  ");
    for (int i = 0; i<2; i++){
      Serial.print(a[i], 2);
      Serial.print('\t');
    }  
    Serial.println(settime);
    */
    delay(100); // หน่วงเวลา 100 มิลลิวินาที 
    settime++;
  }
}

int iy = 1;
void loop() { 
    error();
    Serial.println("----------------------------------");
    Serial.print(iy);
    Serial.print('\t');
    for (int j = 0; j<2; j++){
      long avgt = 0;
      for (int tem = 0; tem<=100; tem++){ avgt+= avg[j][tem]; }
      float tempv = pow(0.9, iy);
      if (    avgt <= 0){  potValue[j] -= tempv;}
      else {   potValue[j] += tempv;}
      Serial.print(avgt);
      Serial.print('\t');
    }
  
    for (int t = 0; t<2; t++) {
        Serial.print(potValue[t], 8);
        Serial.print('\t');
    } Serial.println("\n----------------------------------");
    iy++;
}
