void steer(float angle, float speed)

{

  #define ENA 5
  #define ENB 6
  #define IN1 7
  #define IN2 8
  #define IN3 9
  #define IN4 11
  float speed_left;
  float speed_right;
  speed_left = sin(angle) * speed;
  speed_right = 1-(sin(angle)* speed);

  analogWrite(ENA, speed_left);
  analogWrite(ENB, speed_right);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}

