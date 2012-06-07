
(cl:in-package :asdf)

(defsystem "ardrone_emulator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DroneControl" :depends-on ("_package_DroneControl"))
    (:file "_package_DroneControl" :depends-on ("_package"))
  ))