#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>

namespace gazebo
{
  class ForceMonitorPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      // Найти ссылку (link) pusher
      this->pusherLink = _model->GetLink("pusher");
      if (!this->pusherLink)
      {
        gzerr << "Link 'pusher' not found!" << std::endl;
        return;
      }

      // Найти link tip
      this->tipLink = _model->GetLink("tip");
      if (!this->tipLink)
      {
        gzerr << "Link 'tip' not found!" << std::endl;
        return;
      }

      // Сохранить пороговое значение силы
      this->forceThreshold = -10.0; // Отрицательная сила по Z

      // Сброс флага гравитации при каждом запуске
      this->gravityEnabled = false;

      // Подключить обработчик обновления
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ForceMonitorPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Получить силу, действующую на pusher
      ignition::math::Vector3d force = this->pusherLink->WorldForce();

      // Проверить силу по оси Z
      double forceZ = force.Z();
      if (forceZ < this->forceThreshold) // Проверка на превышение по модулю вниз
      {
        gzdbg << "Force in Z exceeded threshold: " << forceZ << "\n";

        // Включить гравитацию для tip (установить g_value)
        if (!this->gravityEnabled) {
          this->tipLink->SetGravityMode(true); // Включить гравитацию
          this->gravityEnabled = true;
          gzdbg << "Gravity enabled for tip." << std::endl;
        }
      }
    }

  private:
    physics::LinkPtr pusherLink;            // Указатель на pusher
    physics::LinkPtr tipLink;               // Указатель на tip
    event::ConnectionPtr updateConnection; // Обработчик событий обновления
    double forceThreshold;                  // Порог силы
    bool gravityEnabled = false;            // Флаг, показывающий, включена ли гравитация для tip
  };

  // Зарегистрировать плагин
  GZ_REGISTER_MODEL_PLUGIN(ForceMonitorPlugin)
}
