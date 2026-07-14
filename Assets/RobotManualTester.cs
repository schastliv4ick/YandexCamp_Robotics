using UnityEngine;

public class RobotManualTester : MonoBehaviour
{
    [Header("Scripts to Test")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;

    private void Update()
    {
        // 1. Тест движения (Клавиши WASD или стрелочки)
        if (trackController != null)
        {
            trackController.GasInput = Input.GetAxis("Vertical");      // W/S или Стрелки Вверх/Вниз
            trackController.SteerInput = Input.GetAxis("Horizontal");  // A/D или Стрелки Влево/Вправо
        }

        // 2. Тест клешни (Зажмите Пробел, чтобы попытаться схватить мяч)
        if (gripperController != null)
        {
            // Пока пробел зажат — шлем команду зажатия, отпустили — разжатия
            gripperController.GripperCloseCommand = Input.GetKey(KeyCode.Space);
        }
    }
}