using UnityEngine;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Collections.Concurrent;

[System.Serializable]
public class YoloDataPacket
{
    public float angle;      // [-1..1] отклонение центра мяча по X
    public float distance;   // [0..1] нормализованная высота рамки (используется как дистанция)
    public float sees;       // 1.0 – виден, 0.0 – нет
    public float conf;       // уверенность
    public float w;          // нормализованная ширина (0..1)
    public float h;          // нормализованная высота (0..1)
    public float cy;         // нормализованная Y-координата центра (0 – верх, 1 – низ)
}

public class RealVision : MonoBehaviour
{
    [Header("UDP Settings")]
    public int udpPort = 5005;
    public bool useYOLO = false;  // становится true при первом пакете

    [Header("Current Data (read only)")]
    public float normalizedAngle;
    public float normalizedDistance;
    public bool seesBall;
    public float confidence;
    public float ballWidth;      // нормализованная ширина
    public float ballHeight;     // нормализованная высота
    public float normalizedCenterY; // нормализованная Y-координата центра (0 – верх, 1 – низ)

    // Очередь для безопасного обмена между потоками
    private ConcurrentQueue<YoloDataPacket> udpQueue = new ConcurrentQueue<YoloDataPacket>();
    private CancellationTokenSource cts;
    private UdpClient udpClient;

    void Start()
    {
        cts = new CancellationTokenSource();
        try
        {
            udpClient = new UdpClient(new IPEndPoint(IPAddress.Any, udpPort));
            Debug.Log($"[RealVision] UDP‑слушатель запущен на порту {udpPort}");
            // Запускаем фоновую задачу
            Task.Run(() => UdpListenerLoop(cts.Token));
        }
        catch (Exception e)
        {
            Debug.LogError($"[RealVision] Не удалось запустить UDP‑сервер: {e.Message}");
        }
    }

    private async Task UdpListenerLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                var result = await udpClient.ReceiveAsync();
                string json = Encoding.UTF8.GetString(result.Buffer);
                YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);
                if (packet != null)
                {
                    udpQueue.Enqueue(packet);
                }
            }
            catch (ObjectDisposedException)
            {
                // udpClient закрыт – выходим
                break;
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[RealVision] Ошибка приёма UDP: {e.Message}");
            }
        }
    }

    void Update()
    {
        // Обрабатываем все накопленные пакеты (берём последний)
        bool hasPacket = false;
        YoloDataPacket latest = null;
        while (udpQueue.TryDequeue(out var packet))
        {
            latest = packet;
            hasPacket = true;
        }

        if (hasPacket && latest != null)
        {
            useYOLO = true;
            seesBall = latest.sees > 0.5f;
            confidence = latest.conf;
            ballWidth = latest.w;
            ballHeight = latest.h;
            normalizedCenterY = latest.cy;

            if (seesBall)
            {
                normalizedAngle = Mathf.Clamp(latest.angle, -1f, 1f);
                normalizedDistance = Mathf.Clamp01(latest.distance);
            }
            else
            {
                // Если мяч не виден, сохраняем последние значения, но можно сбросить:
                // normalizedAngle = 0f;
                // normalizedDistance = 1f;
                // Оставляем последние, чтобы робот не дёргался
            }
        }
    }

    public float ComputeDistanceToBall(YoloDataPacket data, float imageWidthPixels, 
    float focalLengthPx, float realBallDiameterMeters, float cameraHeightMeters)
    {
        if (data == null || data.sees < 0.5f || data.w < 0.001f)
            return -1f; // не видим мяч

        
        float bboxWidthPx = data.w * imageWidthPixels;
        if (bboxWidthPx < 1f) bboxWidthPx = -1f; // слишком далеко не даем награду 

        float distCam = (realBallDiameterMeters * focalLengthPx) / bboxWidthPx; // гипотенуза

        // Проверка, чтобы под корнем не было отрицательного
        float sq = distCam * distCam - cameraHeightMeters * cameraHeightMeters;
        if (sq < 0f) return -1f; // ошибка измерения

        float horizontalDist = Mathf.Sqrt(sq);
        return horizontalDist;
    }

    void OnDestroy()
    {
        cts?.Cancel();
        udpClient?.Close();
        udpClient?.Dispose();
        Debug.Log("[RealVision] UDP‑слушатель остановлен");
    }
}