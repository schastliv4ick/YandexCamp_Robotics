using System;
using System.IO;
using System.Net;
using System.Text;
using UnityEngine;

public class M2MListener : MonoBehaviour
{
    [Header("Настройки сервера")]
    public int port = 9001;

    [Header("Путь для сохранения JSON")]
    [Tooltip("Оставьте пустым, чтобы сохранять в папку Logs рядом с проектом")]
    public string savePath = "C:/studcamp/YandexCamp_Robotics/Assets"; // например, "C:/Users/ВашеИмя/Desktop/Logs" или "/home/user/Desktop/Logs"

    private HttpListener listener;
    private bool isRunning;

    // Matches the two-command contract that rover_dispatcher.py validates and sends:
    // POST /api/m2m/set_target {"target":"gfsx_yolo","action":"set_target_class","class_name":"..."}
    // POST /api/m2m/activate   {"target":"gfsx_robot","action":"activate","state":"start"}
    [Serializable]
    private class SetTargetCommand
    {
        public string target;
        public string action;
        public string class_name;
    }

    [Serializable]
    private class ActivateCommand
    {
        public string target;
        public string action;
        public string state;
    }

    // Written from the HttpListener callback (a ThreadPool thread) and read from
    // RobotBrain's FixedUpdate (the main thread) — guard with a lock either way.
    private static readonly object activationLock = new object();
    private static bool pendingActivation;
    private static string pendingActivationClassName;

    // Called by RobotBrain while it is idle waiting to be released. Consuming the
    // flag makes each "activate" call start exactly one run.
    public static bool TryConsumePendingActivation(out string targetClassName)
    {
        lock (activationLock)
        {
            if (pendingActivation)
            {
                pendingActivation = false;
                targetClassName = pendingActivationClassName;
                return true;
            }
        }
        targetClassName = null;
        return false;
    }

    void Start()
    {
        StartServer();
    }

    void StartServer()
    {
        try
        {
            listener = new HttpListener();
            listener.Prefixes.Add($"http://*:{port}/api/m2m/");
            listener.Start();
            isRunning = true;
            Debug.Log($"✅ Сервер запущен на порту {port}");
            listener.BeginGetContext(OnRequest, listener);
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ Ошибка: {e.Message}");
        }
    }

    void OnRequest(IAsyncResult ar)
    {
        if (!isRunning) return;
        var context = listener.EndGetContext(ar);
        listener.BeginGetContext(OnRequest, listener);
        System.Threading.ThreadPool.QueueUserWorkItem(_ => Process(context));
    }

    void Process(HttpListenerContext ctx)
    {
        var req = ctx.Request;
        var res = ctx.Response;

        string body = "";
        using (var reader = new StreamReader(req.InputStream, req.ContentEncoding))
            body = reader.ReadToEnd();

        string endpoint = req.Url.AbsolutePath.Replace("/api/m2m/", "").Trim('/');

        // ---- Определяем папку для сохранения ----
        string folder;
        if (!string.IsNullOrEmpty(savePath))
        {
            folder = savePath;
        }
        else
        {
            // По умолчанию: папка Logs рядом с проектом
            folder = Path.Combine(Application.dataPath, "../Logs");
        }

        try
        {
            Directory.CreateDirectory(folder);
            string fileName = $"{endpoint}_{DateTime.Now:yyyy-MM-dd_HH-mm-ss}.json";
            string fullPath = Path.Combine(folder, fileName);
            File.WriteAllText(fullPath, body);

            Debug.Log($"📥 {endpoint} -> {body}");
            Debug.Log($"💾 Сохранён: {fullPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ Ошибка сохранения: {e.Message}");
        }

        HandleM2MCommand(endpoint, body);

        // Ответ
        byte[] buf = Encoding.UTF8.GetBytes("{\"status\":\"ok\"}");
        res.ContentLength64 = buf.Length;
        res.OutputStream.Write(buf, 0, buf.Length);
        res.OutputStream.Close();
    }

    // Runs on the HttpListener ThreadPool callback thread. Only touches thread-safe
    // Unity APIs (Debug.Log) plus the locked static flag above — never scene objects.
    private void HandleM2MCommand(string endpoint, string body)
    {
        switch (endpoint)
        {
            case "set_target":
                HandleSetTarget(body);
                break;
            case "activate":
                HandleActivate(body);
                break;
        }
    }

    private void HandleSetTarget(string body)
    {
        SetTargetCommand cmd;
        try
        {
            cmd = JsonUtility.FromJson<SetTargetCommand>(body);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[M2MListener] set_target: некорректный JSON: {e.Message}");
            return;
        }

        bool isValid = cmd != null
            && cmd.target == "gfsx_yolo"
            && cmd.action == "set_target_class"
            && !string.IsNullOrEmpty(cmd.class_name);

        if (!isValid)
        {
            Debug.LogWarning("[M2MListener] set_target: команда не прошла проверку контракта, игнорируется.");
            return;
        }

        lock (activationLock)
        {
            pendingActivationClassName = cmd.class_name;
        }
        Debug.Log($"[M2MListener] Целевой класс от M2M-эстафеты: {cmd.class_name}");
    }

    private void HandleActivate(string body)
    {
        ActivateCommand cmd;
        try
        {
            cmd = JsonUtility.FromJson<ActivateCommand>(body);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[M2MListener] activate: некорректный JSON: {e.Message}");
            return;
        }

        bool isValid = cmd != null
            && cmd.target == "gfsx_robot"
            && cmd.action == "activate"
            && cmd.state == "start";

        if (!isValid)
        {
            Debug.LogWarning("[M2MListener] activate: команда не соответствует разрешённому контракту " +
                "target=gfsx_robot/action=activate/state=start, игнорируется.");
            return;
        }

        lock (activationLock)
        {
            pendingActivation = true;
        }
        Debug.Log("[M2MListener] ✅ activate/start получен — гусеничный ровер (RobotBrain) будет запущен.");
    }

    void OnDestroy()
    {
        isRunning = false;
        listener?.Stop();
        listener?.Close();
    }
}