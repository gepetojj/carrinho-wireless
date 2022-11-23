class DistanceSensor
{
public:
	DistanceSensor(int trigPin, int echoPin);

	float readDistance();

private:
	int trigPin;
	int echoPin;
};