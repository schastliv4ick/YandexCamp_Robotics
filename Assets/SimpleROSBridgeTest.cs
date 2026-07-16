using UnityEngine;

public class SimpleROSBridgeTest : MonoBehaviour
{
    public ROSBridge rosBridge;
    private float timer;

    void Start()
    {
        if (rosBridge == null)
            rosBridge = GetComponent<ROSBridge>();
    }

    void Update()
    {
        timer += Time.deltaTime;
        
        // Каждые 2 секунды отправляем тестовую команду
        if (timer >= 2f)
        {
            timer = 0f;
            
            // Проверка 1: Движение вперед
            Debug.Log("Test 1: Send Forward (gas=0.5, steering=0)");
            rosBridge.PublishCommand(0.5f, 0f);
        }
        else if (timer >= 1f)
        {
            // Проверка 2: Остановка (проверка Hard Stop)
            Debug.Log("Test 2: Send Stop (0,0) - checking Hard Stop");
            rosBridge.PublishCommand(0f, 0f);
        }
    }
}