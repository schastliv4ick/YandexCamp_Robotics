using UnityEngine;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Collections.Concurrent;

// listen port, parse data, transfer ball coords

public class RealVision : MonoBehaviour {
    [Header("Network Settings")]
    public int udpPort = 5005;

    [Header("State (only for reading in Inspector)")]
    public bool useYOLO = false;

    // telemetry for RobotBrain
    public float normalizedAngle;
    public float normalizedDistance;
    public bool seesBall;

    private CancellationTokenSource cts;
    private ConcurrentQueue<YoloDataPacket> udpQueue = new ConcurrentQueue<YoloDataPacket>();


    [System.Serializable]
    public class YoloDataPacket {
        public float angle;      // Отклонение центра мяча (-1.0 лево, 1.0 право)
        public float distance;   // Высота рамки мяча относительно кадра (0..1)
        public float sees;       // Флаг видимости (1.0 = виден, 0.0 = нет)
        public float conf;       // Уверенность детекции
        public float w;          // Ширина bounding box
        public float h;          // Высота bounding box
    }

    void Start()
    {
        cts = new CancellationTokenSource();
        // Start UDP-listener in the background thread, so it doesn't block the game
        Task.Run(() => UdpListenerLoop(cts.Token));
    }

    private async Task UdpListenerLoop(CancellationToken token)
    {
        using (var udpClient = new UdpClient(udpPort))
        {
            while (!token.IsCancellationRequested)
            {
                try{
                    var result = await udpClient.ReceiveAsync();
                    string json = System.Text.Encoding.UTF8.GetString(result.Buffer);
                    YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);

                    if (packet != null)
                    {
                        udpQueue.Enqueue(packet); // safely push the packet to the queue
                    }
                } catch (ObjectDisposedException) {
                    Debug.LogError($"UDP Listener error: {ex.Message}");
                    break; // exit the loop on error
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[RealVision] UDP package transmission error: {ex.Message}");
                }
            }
        }
    }

    void Update()
    {
        // Process incoming UDP packets
        while (udpQueue.TryDequeue(out YoloDataPacket packet))
        {
            useYOLO = true;
            seesBall = packet.sees > 0.5f;
            
            if (seesBall) {
                // Normalize angle to [-1, 1] range
                normalizedAngle = Mathf.Clamp(packet.angle, -1f, 1f);
                // write normalized distance (the larger the ball, the closer it is)
                normalizedDistance = packet.distance;
                seesBall = packet.sees > 0.5f; // Assuming sees is a float where >0.5 means true
            } else {
                normalizedAngle = 0f;
                normalizedDistance = 1f; // the ball is not seen 
            }
        }
    }

    void OnDestroy()
    {
        cts?.Cancel(); // stop background thread when the object is destroyed
    }
}