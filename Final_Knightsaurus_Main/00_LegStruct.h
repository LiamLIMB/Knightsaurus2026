// Leg Struct:
struct LegState {
  int hip;
  int knee;
  int targetHip;
  int targetKnee;
  uint8_t hipServo;
  uint8_t kneeServo;
  bool active;
};