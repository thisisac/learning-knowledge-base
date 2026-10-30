function imudata = LoadIMUIncreBin(path)
% Load IMU increment binary (double) with 7 columns per record:
% [gpss, dtheta_x(rad), dtheta_y(rad), dtheta_z(rad), dv_x(m/s), dv_y(m/s), dv_z(m/s)]
% returns Nx7 matrix (same layout as existing pipeline expects)

    fid = fopen(path, 'rb');
    assert(fid>0, 'Failed to open IMU binary file: %s', path);
    raw = fread(fid, inf, 'double');
    fclose(fid);

    assert(mod(numel(raw), 7) == 0, 'IMU binary size mismatch (not multiple of 7 doubles).');
    imudata = reshape(raw, 7, []).';
end