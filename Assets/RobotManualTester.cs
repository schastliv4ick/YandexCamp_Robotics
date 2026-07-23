using UnityEngine;
using UnityEngine.InputSystem; // Требует установленного пакета Input System

public class RobotManualTester : MonoBehaviour
{
    [Header("Scripts to Test")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;

    private void Update()
    {
        // Считываем состояние клавиатуры через New Input System
        var keyboard = Keyboard.current;
        if (keyboard == null)
        {
            Debug.LogWarning("[Tester] Клавиатура недоступна!");
            return;
        }

        // 1. Тест движения (Клавиши WASD)
        if (trackController != null)
        {
            float gas = 0f;
            float steer = 0f;

            if (keyboard.wKey.isPressed) gas += 1f;
            if (keyboard.sKey.isPressed) gas -= 1f;
            if (keyboard.aKey.isPressed) steer -= 1f;
            if (keyboard.dKey.isPressed) steer += 1f;

            // Записываем команды ввода в свойства контроллера
            trackController.Move(gas, steer);
        }

        // 2. Тест клешни (Зажмите Пробел, чтобы попытаться схватить мяч)
        if (gripperController != null)
        {
            // Передаем состояние клавиши Space (True, если зажата)
            gripperController.GripperCloseCommand = keyboard.spaceKey.isPressed;
        }
    }
}