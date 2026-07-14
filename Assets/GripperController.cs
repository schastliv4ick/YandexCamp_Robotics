using UnityEngine;

public class GripperController : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Точка фиксации мяча (пустой объект HoldPoint между губками клешни)")]
    [SerializeField] private Transform holdPoint;
    
    [Tooltip("Ссылка на скрипт виртуальных датчиков (для чтения состояния ИК-сенсора)")]
    [SerializeField] private VirtualSensors virtualSensors;

    [Header("Settings")]
    [Tooltip("Тег целевого мяча")]
    [SerializeField] private string targetBallTag = "TargetBall";
    [Tooltip("Радиус зоны захвата для поиска мяча при срабатывании датчика")]
    [SerializeField] private float grabSearchRadius = 0.15f;

    // Внешний флаг управления (устанавливается RL-агентом или скриптом ввода)
    // true = команда «зажать клешню», false = «разжать клешню»
    public bool GripperCloseCommand { get; set; }

    // Текущий статус захвата (удерживаем ли мы мяч)
    public bool IsHolding { get; private set; }

    private GameObject grabbedBall;
    private Rigidbody grabbedBallRb;
    private Collider grabbedBallCollider;
    private Transform originalBallParent;

    private void Update()
    {
        // 1. Логический захват
        // Если пришла команда зажать, ИК-датчик видит мяч (значение > 0.5), и мяч еще не захвачен
        if (GripperCloseCommand && virtualSensors != null && virtualSensors.GripperIRBallDetected > 0.5f && !IsHolding)
        {
            TryGrabBall();
        }

        // 2. Логическое отпускание
        // Если пришла команда разжать (false), но мы удерживаем мяч
        if (!GripperCloseCommand && IsHolding)
        {
            ReleaseBall();
        }
    }

    private void TryGrabBall()
    {
        // Поиск коллайдеров в зоне удержания HoldPoint
        Collider[] hitColliders = Physics.OverlapSphere(holdPoint.position, grabSearchRadius);
        foreach (Collider col in hitColliders)
        {
            if (col.CompareTag(targetBallTag))
            {
                grabbedBall = col.gameObject;
                grabbedBallRb = col.GetComponent<Rigidbody>();
                grabbedBallCollider = col;

                if (grabbedBallRb != null)
                {
                    // Сохраняем исходного родителя мяча, чтобы при отпускании не нарушить иерархию сцены
                    originalBallParent = grabbedBall.transform.parent;

                    // Переводим Rigidbody мяча в режим кинематики (отключаем расчет физики)
                    grabbedBallRb.isKinematic = true;
                    
                    // Отключаем коллайдер мяча, чтобы он не сталкивался с клешней во время переноса
                    grabbedBallCollider.enabled = false;

                    // Перемещаем мяч точно в HoldPoint и делаем его дочерним объектом
                    grabbedBall.transform.position = holdPoint.position;
                    grabbedBall.transform.SetParent(holdPoint);

                    IsHolding = true;
                    break;
                }
            }
        }
    }

    public void ReleaseBall()
    {
        if (grabbedBall == null) return;

        // Возвращаем мяч в исходную иерархию (снимаем родителя)
        grabbedBall.transform.SetParent(originalBallParent);

        // Включаем обратно физический расчет
        if (grabbedBallRb != null)
        {
            grabbedBallRb.isKinematic = false;
        }

        // Включаем обратно коллайдер мяча
        if (grabbedBallCollider != null)
        {
            grabbedBallCollider.enabled = true;
        }

        // Сбрасываем локальные ссылки
        grabbedBall = null;
        grabbedBallRb = null;
        grabbedBallCollider = null;
        originalBallParent = null;
        IsHolding = false;
    }

    // Визуальное отображение зоны поиска в редакторе Unity для удобной настройки
    private void OnDrawGizmosSelected()
    {
        if (holdPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(holdPoint.position, grabSearchRadius);
        }
    }
}