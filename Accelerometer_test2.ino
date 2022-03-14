int xpin = A3;                
int ypin = A2;                
int zpin = A1; 
int xvalue;
int yvalue;
int zvalue;
long sum[3], settime = 1;
float avg[3]; 

void setup()
{
   Serial.begin(9600);          // initialize the serial communications:
}


void loop()
{
  xvalue = analogRead(xpin);                              //reads values from x-pin & measures acceleration in X direction 
  
  yvalue = analogRead(ypin);

  zvalue = analogRead(zpin);
  int temp[3] = {xvalue,yvalue,zvalue};
  for (int i = 0; i<=2; i++){
    sum[i]+=temp[i];
    avg[i] = sum[i]/ settime;
    Serial.print(temp[i]); 
    Serial.print("\tavg: "); 
    Serial.print(avg[i]); 
    Serial.print("\t"); 
  }
  
  Serial.print("\n"); 
  
  delay(100);
  settime++;
}
