
(cl:in-package :asdf)

(defsystem "autonomy_human-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "human" :depends-on ("_package_human"))
    (:file "_package_human" :depends-on ("_package"))
  ))