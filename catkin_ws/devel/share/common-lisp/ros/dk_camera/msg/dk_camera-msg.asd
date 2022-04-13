
(cl:in-package :asdf)

(defsystem "dk_camera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DK_INFO" :depends-on ("_package_DK_INFO"))
    (:file "_package_DK_INFO" :depends-on ("_package"))
  ))