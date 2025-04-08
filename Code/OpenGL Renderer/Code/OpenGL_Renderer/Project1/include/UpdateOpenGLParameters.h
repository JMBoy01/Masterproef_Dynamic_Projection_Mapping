// UpdateOpenGLParameters.h
#ifndef UPDATEOPENGLPARAMETERS_H
#define UPDATEOPENGLPARAMETERS_H

static void UpdateIntrinsics(glm::mat4& projection, const double* data)
{
    // Expected data: [fx, fy, cx, cy, w, h, zNear, zFar, ... ]
    double fx = data[0];
    double fy = data[1];
    double cx = data[2];
    double cy = data[3];
    double w = data[4];
    double h = data[5];
    double zNear = data[6];
    double zFar = data[7];

    cy = h - cy; // cy is now in reference to bottom left corner with y-axis pointing up like opengl standard

    double left = -cx / fx * zNear;
    double right = (w - cx) / fx * zNear;
    double bottom = -cy / fy * zNear;(h - cy) / fy * zNear;
    double top = (h - cy) / fy * zNear;

    projection = glm::frustum(left, right, bottom, top, zNear, zFar);
}

static void UpdatePose(glm::mat4& pose, const double* data)
{ // Transpose matrix when loading by using pose[j][i] -> opencv row major to glm column major
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pose[j][i] = static_cast<float>(data[i * 4 + j]);
        }
    }
}

#endif // UPDATEOPENGLPARAMETERS_H