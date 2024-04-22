def rotate_vec(vec, angle):
    angle = np.deg2rad(angle)
    new_vec = np.copy(vec)
    new_vec[0] = vec[0] * np.cos(angle) - vec[1] * np.sin(angle)
    new_vec[1] = vec[0] * np.sin(angle) + vec[1] * np.cos(angle)
    return new_vec
