
(cl:in-package :asdf)

(defsystem "nao_remote-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SetTransform" :depends-on ("_package_SetTransform"))
    (:file "_package_SetTransform" :depends-on ("_package"))
  ))