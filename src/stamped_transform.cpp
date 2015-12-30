#include "torch-tf.h"

TFIMP(tf::StampedTransform *, StampedTransform, new)(tf::Transform *transform, const char* , const char *)
{
  return self->frame_id_.c_str();
}

TFIMP(void, StampedTransform, delete)(tf::StampedTransform *self)
{
  delete self;
}

TFIMP(const char *, StampedTransform, getFrameId)(tf::StampedTransform *self)
{
  return self->frame_id_.c_str();
}

TFIMP(void, StampedTransform, setFrameId)(tf::StampedTransform *self, const char *id)
{
  self->frame_id_ = id;
}

TFIMP(const char *, StampedTransform, getChildFrameId)(tf::StampedTransform *self)
{
  return self->child_frame_id_.c_str();
}

TFIMP(void, StampedTransform, setChildFrameId)(tf::StampedTransform *self, const char *id)
{
  self->child_frame_id_ = id;
}

TFIMP(tf::Transform *, StampedTransform, getTransform)(tf::StampedTransform *self)
{
  return static_cast<tf::Transform*>(self);
}
