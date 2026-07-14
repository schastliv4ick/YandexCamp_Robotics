using UnityEngine;

public class SimpleWASDMovement : MonoBehaviour
{
    [Header("Настройки движения")]
    [Tooltip("Скорость перемещения модели")]
    public float moveSpeed = 5.0f;

    [Tooltip("Скорость поворота модели в сторону движения (градусов в секунду)")]
    public float rotationSpeed = 720.0f; 

    private void Update()
    {
        // Считываем ввод с клавиатуры:
        // horizontal принимает значения от -1 (клавиша A) до 1 (клавиша D)
        // vertical принимает значения от -1 (клавиша S) до 1 (клавиша W)
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        // Формируем направление движения по осям X и Z
        Vector3 direction = new Vector3(horizontal, 0f, vertical);

        // Если игрок нажал хотя бы одну из клавиш WASD (длина вектора направления больше нуля)
        if (direction.magnitude > 0.1f)
        {
            // Нормализуем вектор, чтобы при движении по диагонали скорость не удваивалась
            direction.Normalize();

            // 1. Двигаем объект в мировом пространстве
            transform.Translate(direction * moveSpeed * Time.deltaTime, Space.World);

            // 2. Рассчитываем поворот в сторону движения
            Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);

            // Плавно разворачиваем модель к целевому углу поворота
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation, 
                targetRotation, 
                rotationSpeed * Time.deltaTime
            );
        }
    }
}