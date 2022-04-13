; Auto-generated. Do not edit!


(cl:in-package dk_camera-msg)


;//! \htmlinclude DK_INFO.msg.html

(cl:defclass <DK_INFO> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform "")
   (angle
    :reader angle
    :initarg :angle
    :type cl:integer
    :initform 0)
   (station_number
    :reader station_number
    :initarg :station_number
    :type cl:integer
    :initform 0))
)

(cl:defclass DK_INFO (<DK_INFO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DK_INFO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DK_INFO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dk_camera-msg:<DK_INFO> is deprecated: use dk_camera-msg:DK_INFO instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <DK_INFO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dk_camera-msg:data-val is deprecated.  Use dk_camera-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <DK_INFO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dk_camera-msg:angle-val is deprecated.  Use dk_camera-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'station_number-val :lambda-list '(m))
(cl:defmethod station_number-val ((m <DK_INFO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dk_camera-msg:station_number-val is deprecated.  Use dk_camera-msg:station_number instead.")
  (station_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DK_INFO>) ostream)
  "Serializes a message object of type '<DK_INFO>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'station_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DK_INFO>) istream)
  "Deserializes a message object of type '<DK_INFO>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'station_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DK_INFO>)))
  "Returns string type for a message object of type '<DK_INFO>"
  "dk_camera/DK_INFO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DK_INFO)))
  "Returns string type for a message object of type 'DK_INFO"
  "dk_camera/DK_INFO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DK_INFO>)))
  "Returns md5sum for a message object of type '<DK_INFO>"
  "a18b9ce4dd51326814ca565d3f1e4457")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DK_INFO)))
  "Returns md5sum for a message object of type 'DK_INFO"
  "a18b9ce4dd51326814ca565d3f1e4457")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DK_INFO>)))
  "Returns full string definition for message of type '<DK_INFO>"
  (cl:format cl:nil "string data~%int32 angle~%int32 station_number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DK_INFO)))
  "Returns full string definition for message of type 'DK_INFO"
  (cl:format cl:nil "string data~%int32 angle~%int32 station_number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DK_INFO>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DK_INFO>))
  "Converts a ROS message object to a list"
  (cl:list 'DK_INFO
    (cl:cons ':data (data msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':station_number (station_number msg))
))
