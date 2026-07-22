using UnityEngine;
using UnityEngine.InputSystem;

public class RobotManualTester : MonoBehaviour
{
    [Header("Scripts to Test")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;

    [Header("ROS Bridge")]
    [SerializeField] private ROSBridge rosBridge;

    private float cameraAngle = 0f;
    private bool gripperClosed = true; // true = закрыта (cmd 2), false = открыта (cmd 1)

    void Update()
    {
        var keyboard = Keyboard.current;
        if (keyboard == null)
        {
            Debug.LogWarning("[Tester] Клавиатура недоступна!");
            return;
        }

        // ===== 1. ДВИЖЕНИЕ (WASD) =====
        float gas = 0f;
        float steer = 0f;
        if (keyboard.wKey.isPressed) steer += 1f;
        if (keyboard.sKey.isPressed) steer -= 1f;
        if (keyboard.aKey.isPressed) gas += 1f;
        if (keyboard.dKey.isPressed) gas -= 1f;

        if (rosBridge != null)
            rosBridge.PublishDrive(gas, steer);

        if (trackController != null)
        {
            trackController.GasInput = gas;
            trackController.SteerInput = steer;
        }

        // ===== 2. КЛЕШНЯ (Пробел) – переключение состояния =====
        if (keyboard.spaceKey.wasPressedThisFrame)
        {
            gripperClosed = !gripperClosed;
            // Отправляем 2 (закрыть) или 1 (открыть) согласно кодам из Python
            rosBridge.PublishGripper(gripperClosed ? 2 : 1);
        }

        if (gripperController != null)
        {
            // Для локальной анимации (если нужна)
            gripperController.GripperCloseCommand = keyboard.spaceKey.isPressed;
        }

        // ===== 3. КАМЕРА (Q/E) – возврат в центр при отпускании =====
        if (keyboard.qKey.isPressed)
            cameraAngle = -1f;
        else if (keyboard.eKey.isPressed)
            cameraAngle = 1f;

        if (rosBridge != null)
            rosBridge.PublishCamera(cameraAngle);
    }
}