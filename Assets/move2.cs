using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

// namespace RosSharp.Control

    public class move_2 : MonoBehaviour
    {
        [SerializeField] float wA1Value = 0.0f;
        [SerializeField] float wA2Value = 0.0f;
        [SerializeField] float wA3Value = 0.0f;
        [SerializeField] float wA4Value = 0.0f;

        public GameObject wheel1;
        public GameObject wheel2;
        public GameObject wheel3;
        public GameObject wheel4;
        // public ControlMode mode = ControlMode.ROS;

        private ArticulationBody wA1;
        private ArticulationBody wA2;
        private ArticulationBody wA3;
        private ArticulationBody wA4;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.07f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 100;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        // ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            wA3 = wheel3.GetComponent<ArticulationBody>();
            wA4 = wheel4.GetComponent<ArticulationBody>();
            SetParameters(wA3);
            SetParameters(wA4);
            // ros = ROSConnection.GetOrCreateInstance();
            // ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
        }

        // void ReceiveROSCmd(TwistMsg cmdVel)
        // {
        //     rosLinear = (float)cmdVel.linear.x;
        //     rosAngular = (float)cmdVel.angular.z;
        //     lastCmdReceived = Time.time;
        // }

        void Update()
        {
            SetSpeed(wA1, wA1Value);
            SetSpeed(wA2, wA2Value);
            SetSpeed(wA3, wA3Value);
            SetSpeed(wA4, wA4Value);
            // KeyBoardUpdate();
            // if (mode == ControlMode.Keyboard)
            // {
                
            // }
            // else if (mode == ControlMode.ROS)
            // {
            //     ROSUpdate();
            // }     
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            
            joint.xDrive = drive;
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirction = Input.GetAxis("Horizontal");
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotInput(inputSpeed, inputRotationSpeed);
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            Debug.Log(wheel1Rotation);
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }
    }

