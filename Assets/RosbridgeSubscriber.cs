using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using WebSocketSharp;

public class RosbridgeSubscriber : MonoBehaviour
{
    public string rosbridgeServerAddress = "ws://localhost:9090";
    public string topicName = "/robot_news1"; 

    private WebSocket ws;
    
    [SerializeField] float front_right_value = 0.0f;
    [SerializeField] float front_left_value = 0.0f;
    [SerializeField] float back_right_value = 0.0f;
    [SerializeField] float back_left_value = 0.0f;
    
    public GameObject front_right_object;
    public GameObject front_left_object;
    public GameObject back_right_object;
    public GameObject back_left_object;
    
    private ArticulationBody front_right;
    private ArticulationBody front_left;
    private ArticulationBody back_right;
    private ArticulationBody back_left;
    public float maxLinearSpeed = 2; //  m/s
    public float maxRotationalSpeed = 1;//
    public float wheelRadius = 0.07f; //meters
    public float trackWidth = 0.288f; // meters Distance between tyres
    public float forceLimit = 100;
    public float damping = 10;
    private RotationDirection direction;
    void Start()
    {
        ws = new WebSocket(rosbridgeServerAddress);

        ws.OnMessage += OnWebSocketMessage;

        ws.Connect();

        SubscribeToTopic(topicName);

        front_right = front_right_object.GetComponent<ArticulationBody>();
        front_left = front_left_object.GetComponent<ArticulationBody>();
        back_right = back_right_object.GetComponent<ArticulationBody>();
        back_left = back_left_object.GetComponent<ArticulationBody>();
        SetParameters(front_right);
        SetParameters(front_left);
        SetParameters(back_right);
        SetParameters(back_left);
    }

    void Update()
    {
        SetSpeed(front_right, front_right_value);
        SetSpeed(front_left, front_left_value);
        SetSpeed(back_right, back_right_value);
        SetSpeed(back_left, back_left_value);
    }

    private void OnDestroy()
    {
        if (ws != null && ws.IsAlive)
        {
            ws.Close();
        }
    }

    private void OnWebSocketMessage(object sender, MessageEventArgs e)
    {
        Debug.Log("Received message: " + e.Data);
        string jsonString = e.Data;

        // 解析JSON字符串为RobotNewsMessage对象
        RobotNewsMessage message = JsonUtility.FromJson<RobotNewsMessage>(jsonString);

        // 提取"data"数组
        float[] data = message.msg.data;
	Debug.Log(data);
        front_right_value = data[0];
        front_left_value = data[1];
        back_right_value = data[2];
        back_left_value = data[3];
        ;
    }

    // 发送订阅请求
    private void SubscribeToTopic(string topic)
    {
        // 创建订阅请求消息
        string subscribeMessage = "{\"op\":\"subscribe\",\"id\":\"1\",\"topic\":\"" + topic + "\",\"type\":\"std_msgs/msg/Float32MultiArray\"}";

        // 发送订阅请求消息到Rosbridge服务器
        ws.Send(subscribeMessage);
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
        drive.targetVelocity = wheelSpeed;
        joint.xDrive = drive;
    }


    [System.Serializable]
    public class RobotNewsMessage
    {
        public string op;
        public string topic;
        public MessageData msg;
    }

    [System.Serializable]
    public class MessageData
    {
        public LayoutData layout;
        public float[] data;
    }

    [System.Serializable]
    public class LayoutData
    {
        public int[] dim;
        public int data_offset;
    }

}


