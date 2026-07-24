using UnityEngine;
using System;
using System.Collections.Concurrent;
using Stopwatch = System.Diagnostics.Stopwatch;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

[Serializable]
public class YoloDataPacket
{
    public int version;
    public int seq;
    public double timestamp;
    public int frame_w;
    public int frame_h;

    public float angle;      // legacy normalized X offset [-1, 1]
    public float distance;   // legacy normalized box height [0, 1]
    public float sees;
    public float conf;
    public float w;          // normalized bounding-box width [0, 1]
    public float h;          // normalized bounding-box height [0, 1]
    public float cx;         // normalized bounding-box center X [0, 1]
    public float cy;         // normalized bounding-box center Y [0, 1]
    public bool clipped;
}

public class RealVision : MonoBehaviour
{
    public const int SupportedProtocolVersion = 1;

    [Header("UDP Settings")]
    [SerializeField, Min(1)] private int udpPort = 5005;
    [SerializeField, Min(0.05f)] private float packetTimeoutSeconds = 0.5f;

    [Header("Current Data (read only)")]
    public bool useYOLO;
    public bool seesBall;
    public float confidence;
    public float normalizedAngle;
    public float normalizedDistance;
    public float ballWidth;
    public float ballHeight;
    public float normalizedCenterX = 0.5f;
    public float normalizedCenterY = 0.5f;
    public bool boundingBoxClipped;
    public int frameWidthPixels;
    public int frameHeightPixels;
    public int lastPacketSequence = -1;

    private readonly ConcurrentQueue<ReceivedPacket> packetQueue = new ConcurrentQueue<ReceivedPacket>();
    private CancellationTokenSource cancellation;
    private UdpClient udpClient;
    private Task listenerTask;
    private YoloDataPacket latestPacket;
    private long latestArrivalTimestamp;
    private int rejectedPacketCount;

    private struct ReceivedPacket
    {
        public YoloDataPacket Packet;
        public long ArrivalTimestamp;
    }

    public int UdpPort => udpPort;
    public bool HasFreshPacket => latestPacket != null && PacketAgeSeconds <= packetTimeoutSeconds;
    public float PacketAgeSeconds => GetPacketAgeSeconds(latestArrivalTimestamp);

    private void Start()
    {
        StartListener();
    }

    private void Update()
    {
        PromoteNewestPacket();

        if (latestPacket != null && PacketAgeSeconds > packetTimeoutSeconds)
        {
            useYOLO = false;
            seesBall = false;
        }
    }

    private void StartListener()
    {
        cancellation = new CancellationTokenSource();

        try
        {
            udpClient = new UdpClient(new IPEndPoint(IPAddress.Any, udpPort));
            listenerTask = Task.Run(() => ListenerLoop(cancellation.Token));
            Debug.Log($"[RealVision] UDP listener started on port {udpPort}, protocol v{SupportedProtocolVersion}.");
        }
        catch (Exception exception)
        {
            useYOLO = false;
            Debug.LogError($"[RealVision] Failed to start UDP listener on port {udpPort}: {exception.Message}");
        }
    }

    private async Task ListenerLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                UdpReceiveResult result = await udpClient.ReceiveAsync();
                string json = Encoding.UTF8.GetString(result.Buffer);
                YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);

                if (!IsValidPacket(packet))
                    continue;

                packetQueue.Enqueue(new ReceivedPacket
                {
                    Packet = packet,
                    ArrivalTimestamp = Stopwatch.GetTimestamp()
                });
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (SocketException exception)
            {
                if (!token.IsCancellationRequested)
                    Debug.LogWarning($"[RealVision] UDP receive error: {exception.Message}");
            }
            catch (Exception exception)
            {
                if (!token.IsCancellationRequested)
                    Debug.LogWarning($"[RealVision] Invalid UDP packet: {exception.Message}");
            }
        }
    }

    private bool IsValidPacket(YoloDataPacket packet)
    {
        bool valid = packet != null
            && packet.version == SupportedProtocolVersion
            && packet.frame_w > 0
            && packet.frame_h > 0
            && IsFinite(packet.sees)
            && IsFinite(packet.conf)
            && IsFinite(packet.w)
            && IsFinite(packet.h)
            && IsFinite(packet.cx)
            && IsFinite(packet.cy)
            && packet.conf >= 0f
            && packet.conf <= 1f
            && packet.w >= 0f
            && packet.w <= 1f
            && packet.h >= 0f
            && packet.h <= 1f
            && packet.cx >= 0f
            && packet.cx <= 1f
            && packet.cy >= 0f
            && packet.cy <= 1f;

        if (!valid && rejectedPacketCount < 5)
        {
            rejectedPacketCount++;
            int receivedVersion = packet != null ? packet.version : -1;
            Debug.LogWarning(
                $"[RealVision] Rejected UDP packet #{rejectedPacketCount}. "
                + $"Expected protocol v{SupportedProtocolVersion}, received v{receivedVersion}. "
                + "Run the matching yolo_vision_node.py.");
        }

        return valid;
    }

    private void PromoteNewestPacket()
    {
        ReceivedPacket newest = default(ReceivedPacket);
        bool found = false;

        while (packetQueue.TryDequeue(out ReceivedPacket received))
        {
            newest = received;
            found = true;
        }

        if (!found)
            return;

        ApplyPacket(newest.Packet, newest.ArrivalTimestamp);
    }

    private void ApplyPacket(YoloDataPacket packet, long arrivalTimestamp)
    {
        latestPacket = packet;
        latestArrivalTimestamp = arrivalTimestamp;

        useYOLO = true;
        seesBall = packet.sees > 0.5f;
        confidence = Mathf.Clamp01(packet.conf);
        normalizedAngle = Mathf.Clamp(packet.angle, -1f, 1f);
        normalizedDistance = Mathf.Clamp01(packet.distance);
        ballWidth = Mathf.Clamp01(packet.w);
        ballHeight = Mathf.Clamp01(packet.h);
        normalizedCenterX = Mathf.Clamp01(packet.cx);
        normalizedCenterY = Mathf.Clamp01(packet.cy);
        boundingBoxClipped = packet.clipped;
        frameWidthPixels = packet.frame_w;
        frameHeightPixels = packet.frame_h;
        lastPacketSequence = packet.seq;
    }

    public bool TryGetLatestPacket(out YoloDataPacket packet)
    {
        // Inspector context-menu actions may run before Update() consumes the UDP queue.
        PromoteNewestPacket();

        if (!HasFreshPacket)
        {
            packet = null;
            return false;
        }

        packet = latestPacket;
        return true;
    }

    public float GetHorizontalBearingDegrees(float focalLengthPx, float fallbackFrameWidthPixels)
    {
        if (!TryGetLatestPacket(out YoloDataPacket packet) || packet.sees <= 0.5f)
            return 0f;

        float frameWidth = packet.frame_w > 0 ? packet.frame_w : fallbackFrameWidthPixels;
        if (frameWidth <= 0f || focalLengthPx <= 0f)
            return 0f;

        float offsetPixels = (packet.cx - 0.5f) * frameWidth;
        return Mathf.Atan(offsetPixels / focalLengthPx) * Mathf.Rad2Deg;
    }

    public float GetCalibratedNormalizedAngle(float focalLengthPx, float fallbackFrameWidthPixels)
    {
        if (!TryGetLatestPacket(out YoloDataPacket packet) || packet.sees <= 0.5f)
            return 0f;

        float frameWidth = packet.frame_w > 0 ? packet.frame_w : fallbackFrameWidthPixels;
        if (frameWidth <= 0f || focalLengthPx <= 0f)
            return Mathf.Clamp(packet.angle, -1f, 1f);

        float halfFovDegrees = Mathf.Atan(0.5f * frameWidth / focalLengthPx) * Mathf.Rad2Deg;
        if (halfFovDegrees <= 0.001f)
            return 0f;

        return Mathf.Clamp(GetHorizontalBearingDegrees(focalLengthPx, frameWidth) / halfFovDegrees, -1f, 1f);
    }

    public static float ComputeHorizontalDistanceToBall(
        YoloDataPacket packet,
        float fallbackFrameWidthPixels,
        float focalLengthPx,
        float realBallDiameterMeters,
        float cameraHeightMeters,
        float distanceScale,
        float distanceOffsetMeters)
    {
        if (packet == null
            || packet.sees <= 0.5f
            || packet.clipped
            || focalLengthPx <= 0f
            || realBallDiameterMeters <= 0f)
        {
            return -1f;
        }

        float frameWidth = packet.frame_w > 0 ? packet.frame_w : fallbackFrameWidthPixels;
        float bboxWidthPixels = packet.w * frameWidth;
        if (frameWidth <= 0f || bboxWidthPixels < 1f)
            return -1f;

        float cameraToBallCenterDistance = realBallDiameterMeters * focalLengthPx / bboxWidthPixels;
        float ballCenterHeight = realBallDiameterMeters * 0.5f;
        float verticalDifference = cameraHeightMeters - ballCenterHeight;
        float horizontalSquared = cameraToBallCenterDistance * cameraToBallCenterDistance
            - verticalDifference * verticalDifference;

        if (horizontalSquared <= 0f)
            return -1f;

        float horizontalDistance = Mathf.Sqrt(horizontalSquared);
        return Mathf.Max(0f, horizontalDistance * distanceScale + distanceOffsetMeters);
    }

    public static bool TryCalibrateFocalLength(
        YoloDataPacket packet,
        float fallbackFrameWidthPixels,
        float realBallDiameterMeters,
        float cameraHeightMeters,
        float knownHorizontalDistanceMeters,
        out float focalLengthPx)
    {
        focalLengthPx = -1f;

        if (packet == null
            || packet.sees <= 0.5f
            || packet.clipped
            || realBallDiameterMeters <= 0f
            || knownHorizontalDistanceMeters <= 0f)
        {
            return false;
        }

        float frameWidth = packet.frame_w > 0 ? packet.frame_w : fallbackFrameWidthPixels;
        float bboxWidthPixels = packet.w * frameWidth;
        if (frameWidth <= 0f || bboxWidthPixels < 1f)
            return false;

        float ballCenterHeight = realBallDiameterMeters * 0.5f;
        float verticalDifference = cameraHeightMeters - ballCenterHeight;
        float cameraToBallCenterDistance = Mathf.Sqrt(
            knownHorizontalDistanceMeters * knownHorizontalDistanceMeters
            + verticalDifference * verticalDifference);

        focalLengthPx = bboxWidthPixels * cameraToBallCenterDistance / realBallDiameterMeters;
        return IsFinite(focalLengthPx) && focalLengthPx > 0f;
    }

    private static float GetPacketAgeSeconds(long arrivalTimestamp)
    {
        if (arrivalTimestamp <= 0L)
            return float.PositiveInfinity;

        long elapsedTicks = Stopwatch.GetTimestamp() - arrivalTimestamp;
        if (elapsedTicks <= 0L)
            return 0f;

        return (float)(elapsedTicks / (double)Stopwatch.Frequency);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void OnDestroy()
    {
        cancellation?.Cancel();
        udpClient?.Close();
        udpClient?.Dispose();

        try
        {
            listenerTask?.Wait(100);
        }
        catch (AggregateException)
        {
            // Shutdown race; socket is already disposed.
        }

        Debug.Log("[RealVision] UDP listener stopped.");
    }
}
