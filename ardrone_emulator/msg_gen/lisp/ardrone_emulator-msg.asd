
(cl:in-package :asdf)

(defsystem "ardrone_emulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NavData" :depends-on ("_package_NavData"))
    (:file "_package_NavData" :depends-on ("_package"))
  ))