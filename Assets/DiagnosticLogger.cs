using UnityEngine;
using Unity.MLAgents;
using System.IO;
using System.Text;
using System.Globalization; // Обязательно для InvariantCulture!

/// <summary>
/// Два независимых механизма логирования, живущих в одном компоненте:
///
/// 1) Пошаговый CSV (diagnostic_log.csv) — точно по П6, Шаг 1.
///    ТОЛЬКО для одиночного локального тестового прогона (enableLogging=true).
///    НЕ включайте это во время реального обучения с --num-envs > 1 — все
///    параллельные процессы среды будут пытаться писать в один и тот же файл
///    на диске и испортят его.
///
/// 2) Метрика по эпизодам (кастомная, "в каких сценариях робот схватил мяч") —
///    идёт через Academy.Instance.StatsRecorder прямо в TensorBoard. Это
///    безопасно работает даже с 16 параллельными средами в облаке, потому что
///    StatsRecorder агрегирует данные централизованно через Academy, а не
///    пишет в файл сам. CSV-дублирование этой метрики (episode_metrics.csv)
///    держите включённым только для одиночных локальных прогонов
///    (writeEpisodeCsvLocally=true), по той же причине, что и в пункте 1.
/// </summary>
public class DiagnosticLogger : MonoBehaviour
{
    [Header("1. Пошаговый лог (P6, Шаг 1) — только ОДИНОЧНЫЙ локальный прогон!")]
    [Tooltip("НЕ включайте во время облачного обучения с --num-envs > 1.")]
    public bool enableLogging = false;
    [Tooltip("Записывать каждый N-й шаг (1 = каждый)")]
    public int logEveryN = 1;
    [Tooltip("Ограничение размера файла (строк)")]
    public int maxRows = 2000;

    [Header("2. Метрика по эпизодам (кастомная) — можно держать включённой всегда")]
    public bool enableEpisodeMetrics = true;
    [Tooltip("CSV по эпизодам пишем ТОЛЬКО при локальном одиночном прогоне. " +
             "В облаке (--num-envs>1) выключайте — TensorBoard всё равно получит метрику через StatsRecorder.")]
    public bool writeEpisodeCsvLocally = true;

    private StreamWriter stepWriter;
    private int rowsWritten = 0;
    private int stepLogCounter = 0;
    private float startTime;

    private StreamWriter episodeWriter;
    private int episodeCounter = 0;

    void Start()
    {
        startTime = Time.time;

        if (enableLogging)
        {
            // Путь к файлу: на один уровень выше папки Assets
            string path = Path.Combine(Application.dataPath, "..", "diagnostic_log.csv");
            stepWriter = new StreamWriter(path, false, Encoding.UTF8);
            stepWriter.WriteLine("time,step,ballSeen,ballAngle,ballDist,uz,irL,irR,gripIR,camYaw,gas,steering,hasBall,holdTicks,isRetrying,displacementX,displacementZ,heading,speed");
            Debug.Log($"[DiagnosticLogger] Пошаговый лог запущен: {path}");
        }

        if (enableEpisodeMetrics && writeEpisodeCsvLocally)
        {
            string epPath = Path.Combine(Application.dataPath, "..", "episode_metrics.csv");
            episodeWriter = new StreamWriter(epPath, false, Encoding.UTF8);
            episodeWriter.WriteLine("episodeIndex,outcome,totalSteps,robotMass,ballMassMultiplier,ballScaleMultiplier,actionLatency,elapsedTime");
            Debug.Log($"[DiagnosticLogger] Лог по эпизодам запущен: {epPath}");
        }
    }

    void OnDestroy()
    {
        stepWriter?.Close();
        episodeWriter?.Close();
    }

    /// <summary>Точно по П6, Шаг 1 — одна строка телеметрии за шаг принятия решения.</summary>
    public void LogStep(
        int step, bool ballSeen, float ballAngle, float ballDist,
        float uz, int irL, int irR, int gripIR, float camYaw,
        float gas, float steering, bool hasBall, int holdTicks,
        bool isRetrying, float displacementX, float displacementZ,
        float heading, float speed)
    {
        if (!enableLogging || stepWriter == null || rowsWritten >= maxRows) return;

        // Прореживание по logEveryN — считаем независимо от общего числа записанных строк
        stepLogCounter++;
        if (stepLogCounter % Mathf.Max(1, logEveryN) != 0) return;

        float elapsed = Time.time - startTime;

        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1},{2},{3:F4},{4:F4},{5:F4},{6},{7},{8},{9:F4},{10:F4},{11:F4},{12},{13},{14},{15:F4},{16:F4},{17:F4},{18:F4}",
            elapsed, step, ballSeen ? 1 : 0, ballAngle, ballDist, uz, irL, irR, gripIR, camYaw,
            gas, steering, hasBall ? 1 : 0, holdTicks, isRetrying ? 1 : 0,
            displacementX, displacementZ, heading, speed);

        stepWriter.WriteLine(line);
        stepWriter.Flush(); // Сбрасываем буфер, чтобы данные не пропали при сбое/остановке
        rowsWritten++;

        if (rowsWritten >= maxRows)
            Debug.Log($"[DiagnosticLogger] Сбор пошагового лога завершён. Достигнут лимит {maxRows} строк.");
    }

    /// <summary>
    /// Кастомная метрика: вызывать РОВНО ОДИН РАЗ на каждое завершение эпизода,
    /// с указанием исхода ("success" / "fall" / "timeout") и активных на этот
    /// эпизод параметров Domain Randomization — чтобы видеть, при каких
    /// сценариях робот реально хватает мяч, а при каких — нет.
    /// </summary>
    public void LogEpisodeEnd(string outcome, int totalSteps, float robotMass,
        float ballMassMultiplier, float ballScaleMultiplier, int actionLatency)
    {
        if (!enableEpisodeMetrics) return;
        episodeCounter++;

        // --- 1) TensorBoard через Academy.StatsRecorder ---
        // Безопасно даже с 16 параллельными средами: каждый процесс отправляет
        // данные через связь с Academy, а агрегацию и запись делает сам ML-Agents.
        var stats = Academy.Instance.StatsRecorder;

        // Общий success rate: TensorBoard усредняет это значение внутри окна
        // summary_freq из config.yaml — итоговая линия и есть процент успеха.
        stats.Add("Custom/GrabSuccessRate", outcome == "success" ? 1f : 0f);

        // Разбивка по сценарию: сравниваем средние параметры DR отдельно для
        // успешных эпизодов и для всех остальных (fall/timeout). Если линия
        // "OnSuccess" заметно ниже/выше "OnOther" — это и есть ответ на вопрос
        // "при каких сценариях робот реально хватает мяч".
        string suffix = outcome == "success" ? "OnSuccess" : "OnOther";
        stats.Add($"Custom/RobotMass_{suffix}", robotMass);
        stats.Add($"Custom/BallMassMultiplier_{suffix}", ballMassMultiplier);
        stats.Add($"Custom/ActionLatencySteps_{suffix}", actionLatency);

        // --- 2) CSV — только для локального одиночного прогона ---
        if (writeEpisodeCsvLocally && episodeWriter != null)
        {
            float elapsed = Time.time - startTime;
            string line = string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},{3:F3},{4:F3},{5:F3},{6},{7:F3}",
                episodeCounter, outcome, totalSteps, robotMass, ballMassMultiplier,
                ballScaleMultiplier, actionLatency, elapsed);
            episodeWriter.WriteLine(line);
            episodeWriter.Flush();
        }
    }
}
