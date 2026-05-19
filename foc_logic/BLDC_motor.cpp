class BLDCMotor
{
private:
	/* data */
public:
	float speed = 0.0f;
	float angle = 0.0f;
	float inertia = 0.001f;
	float kv = 0.1f; // Torque constant (Nm/A)

	BLDCMotor(/* args */);
	~BLDCMotor();
};

BLDCMotor::BLDCMotor(/* args */)
{
}

BLDCMotor::~BLDCMotor()
{
}
