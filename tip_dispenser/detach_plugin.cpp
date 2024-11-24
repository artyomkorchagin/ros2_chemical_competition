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

      // Найти joint body_tip
      this->bodyTipJoint = _model->GetJoint("body_tip");
      if (!this->bodyTipJoint)
      {
        gzerr << "Joint 'body_tip' not found!" << std::endl;
        return;
      }

      // Сохранить пороговое значение силы
      this->forceThreshold = -10.0; // Отрицательная сила по Z

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

        // Отключить joint body_tip
        this->bodyTipJoint->Detach();

        // Можно добавить дополнительную логику (например, отключить только один раз)
        this->updateConnection.reset(); // Остановить проверку после отключения
      }
    }

  private:
    physics::LinkPtr pusherLink;            // Указатель на pusher
    physics::JointPtr bodyTipJoint;         // Указатель на body_tip
    event::ConnectionPtr updateConnection; // Обработчик событий обновления
    double forceThreshold;                  // Порог силы
  };

  // Зарегистрировать плагин
  GZ_REGISTER_MODEL_PLUGIN(ForceMonitorPlugin)
}

