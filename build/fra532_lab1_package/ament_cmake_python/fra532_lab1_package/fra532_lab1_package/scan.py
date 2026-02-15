#!/usr/bin/python3

import numpy as np
from scipy.spatial import KDTree


def data_association(previous_points, current_points, max_corr_dist=None):
    """
    Find nearest neighbors between point clouds
    
    Args:
        previous_points: Reference point cloud (N, 2)
        current_points: Query point cloud (M, 2)
        max_corr_dist: Maximum correspondence distance (optional)
    
    Returns:
        current_matched: Matched points from current
        previous_matched: Corresponding points from previous
        distances: Distances between matches
        indices: Indices of matches in previous_points
    """
    tree = KDTree(previous_points)
    dists, idx = tree.query(current_points, k=1)
    previous_points_match = previous_points[idx, :]

    if max_corr_dist is not None:
        mask = dists < max_corr_dist
        return current_points[mask], previous_points_match[mask], dists[mask], idx[mask]
    
    return current_points, previous_points_match, dists, idx


def icp_matching_point2point(previous_points, current_points, 
                init_x=None, init_y=None, init_theta=None,
                MAX_ITERATION=30, ERROR_BOUND=0.001, max_corr_dist=None):
    """
    Iterative Closest Point (ICP) for 2D point cloud alignment
    
    Args:
        previous_points: Reference point cloud (target) in map/odom frame, shape (N, 2)
        current_points: Query point cloud (source) in base_link frame OR already transformed, shape (M, 2)
        init_x, init_y, init_theta: Initial guess for transformation (optional)
        MAX_ITERATION: Maximum number of iterations
        ERROR_BOUND: Convergence threshold for error reduction
        max_corr_dist: Maximum distance for point correspondences (optional)
    
    Returns:
        x, y, theta: Final transformation (absolute pose in map frame)
        count: Number of iterations
        error: Final mean error
        success: Whether ICP converged successfully
    
    Notes:
        - If init_x/y/theta are None, assumes current_points is already in map frame
        - If init_x/y/theta are provided, applies initial transformation first
        - Returns ABSOLUTE pose, not incremental transform
    """
    
    # Copy to avoid modifying input
    prev_pts = previous_points.copy()
    curr_pts = current_points.copy()
    
    # Initialize transformation
    if init_x is not None and init_y is not None and init_theta is not None:
        # Apply initial guess ONCE at the beginning
        c, s = np.cos(init_theta), np.sin(init_theta)
        R_init = np.array([[c, -s], [s, c]])
        t_init = np.array([init_x, init_y])
        
        # Transform current points to initial pose
        curr_pts = (R_init @ curr_pts.T).T + t_init
        
        # Start with initial transformation
        R_total = R_init.copy()
        t_total = t_init.copy()
    else:
        # No initial guess - assume current_points already transformed
        R_total = np.eye(2)
        t_total = np.array([0.0, 0.0])
    
    prevError = np.inf
    count = 0
    
    for iteration in range(MAX_ITERATION):
        count += 1
        
        # 1) Find correspondences
        src, tgt, dists, indices = data_association(prev_pts, curr_pts, max_corr_dist=max_corr_dist)
        
        # Check if enough correspondences
        if src.shape[0] < 3:
            # Not enough matches
            success = False
            error = prevError
            break
        

        # 2) Calculate error
        keep = int(0.8 * len(dists))
        idx_sort = np.argsort(dists)
        sel = idx_sort[:keep]
        src = src[sel]; tgt = tgt[sel]; dists = dists[sel]
        error = np.mean(dists)
        
        # 3) Check convergence
        if abs(prevError - error) < ERROR_BOUND:
            success = True
            break
        
        prevError = error
        
        # 4) Compute transformation using SVD (Kabsch algorithm)
        src_centroid = src.mean(axis=0)
        tgt_centroid = tgt.mean(axis=0)
        
        # Center the points
        src_centered = src - src_centroid
        tgt_centered = tgt - tgt_centroid
        
        # Compute cross-covariance matrix
        H = src_centered.T @ tgt_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        
        # Compute rotation (with reflection handling)
        d = np.linalg.det(Vt.T @ U.T)
        R_inc = Vt.T @ np.diag([1.0, np.sign(d)]) @ U.T
        
        # Compute translation
        t_inc = tgt_centroid - R_inc @ src_centroid
        
        # 5) Apply incremental transformation to current points
        curr_pts = (R_inc @ curr_pts.T).T + t_inc
        
        # 6) Accumulate total transformation
        R_total = R_inc @ R_total
        t_total = R_inc @ t_total + t_inc
    
    # Check if we reached max iterations without converging
    if count >= MAX_ITERATION:
        success = error < ERROR_BOUND
    
    # Extract final pose
    x = t_total[0]
    y = t_total[1]
    theta = np.arctan2(R_total[1, 0], R_total[0, 0])
    
    return x, y, theta, count, error, success


# ============================================================================
# POINT-TO-PLANE ICP FUNCTIONS
# ============================================================================

def compute_normals(points, k=5):
    """
    Compute normal vectors for each point using local neighborhood.
    
    Uses PCA on k-nearest neighbors to estimate the local surface normal.
    
    Args:
        points: Point cloud (N, 2)
        k: Number of neighbors to use for normal estimation
    
    Returns:
        normals: Normal vectors (N, 2), unit length
    """
    if points.shape[0] < k:
        # Not enough points, return default normals
        return np.zeros_like(points)
    
    tree = KDTree(points)
    normals = np.zeros_like(points)
    
    for i in range(points.shape[0]):
        # Find k nearest neighbors
        dists, indices = tree.query(points[i], k=k)
        neighbors = points[indices]
        
        # Compute centroid of neighbors
        centroid = neighbors.mean(axis=0)
        
        # Center the neighbors
        centered = neighbors - centroid
        
        # Compute covariance matrix
        cov = centered.T @ centered
        
        # Eigenvalue decomposition
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # Normal is eigenvector corresponding to smallest eigenvalue
        # (direction of least variance)
        normal = eigenvectors[:, 0]
        
        # Ensure consistent orientation (pointing outward from centroid)
        # For 2D scans from a robot, normals should generally point away from origin
        if np.dot(normal, centroid) < 0:
            normal = -normal
        
        normals[i] = normal
    
    return normals


def data_association_point2plane(previous_points, current_points, previous_normals, max_corr_dist=None):
    """
    Find nearest neighbors with normal information for point-to-plane ICP.
    
    This is similar to data_association but also returns the normals
    of the matched target points.
    
    Args:
        previous_points: Reference point cloud (N, 2)
        current_points: Query point cloud (M, 2)
        previous_normals: Normal vectors for previous_points (N, 2)
        max_corr_dist: Maximum correspondence distance (optional)
    
    Returns:
        current_matched: Matched points from current
        previous_matched: Corresponding points from previous
        normals_matched: Normal vectors of matched previous points
        distances: Distances between matches
        indices: Indices of matches in previous_points
    """
    tree = KDTree(previous_points)
    dists, idx = tree.query(current_points, k=1)
    previous_points_match = previous_points[idx, :]
    normals_match = previous_normals[idx, :]

    if max_corr_dist is not None:
        mask = dists < max_corr_dist
        return (current_points[mask], previous_points_match[mask], 
                normals_match[mask], dists[mask], idx[mask])
    
    return current_points, previous_points_match, normals_match, dists, idx


def icp_matching_point2plane(previous_points, current_points, 
                             init_x=None, init_y=None, init_theta=None,
                             MAX_ITERATION=30, ERROR_BOUND=0.001, max_corr_dist=None,
                             normal_k=5):
    """
    Point-to-Plane ICP for 2D point cloud alignment.
    
    Unlike point-to-point ICP which minimizes point-to-point distances,
    this minimizes point-to-plane distances, which is more robust to
    noise and provides better convergence.
    
    Args:
        previous_points: Reference point cloud (target) in map/odom frame, shape (N, 2)
        current_points: Query point cloud (source) in base_link frame OR already transformed, shape (M, 2)
        init_x, init_y, init_theta: Initial guess for transformation (optional)
        MAX_ITERATION: Maximum number of iterations
        ERROR_BOUND: Convergence threshold for error reduction
        max_corr_dist: Maximum distance for point correspondences (optional)
        normal_k: Number of neighbors for normal estimation
    
    Returns:
        x, y, theta: Final transformation (absolute pose in map frame)
        count: Number of iterations
        error: Final mean error
        success: Whether ICP converged successfully
    
    Notes:
        - Computes normals for the target (previous) point cloud
        - Minimizes point-to-plane distance instead of point-to-point
        - Generally more robust and faster convergence than point-to-point
    """
    
    # Copy to avoid modifying input
    prev_pts = previous_points.copy()
    curr_pts = current_points.copy()
    
    # Compute normals for target point cloud
    prev_normals = compute_normals(prev_pts, k=normal_k)
    
    # Initialize transformation
    if init_x is not None and init_y is not None and init_theta is not None:
        # Apply initial guess ONCE at the beginning
        c, s = np.cos(init_theta), np.sin(init_theta)
        R_init = np.array([[c, -s], [s, c]])
        t_init = np.array([init_x, init_y])
        
        # Transform current points to initial pose
        curr_pts = (R_init @ curr_pts.T).T + t_init
        
        # Start with initial transformation
        R_total = R_init.copy()
        t_total = t_init.copy()
    else:
        # No initial guess - assume current_points already transformed
        R_total = np.eye(2)
        t_total = np.array([0.0, 0.0])
    
    prevError = np.inf
    count = 0
    
    for iteration in range(MAX_ITERATION):
        count += 1
        
        # 1) Find correspondences with normals
        src, tgt, normals, dists, indices = data_association_point2plane(
            prev_pts, curr_pts, prev_normals, max_corr_dist=max_corr_dist
        )
        
        # Check if enough correspondences
        if src.shape[0] < 3:
            # Not enough matches
            success = False
            error = prevError
            break
        
        # 2) Calculate point-to-plane error
        # Error is the distance along the normal direction
        diff = src - tgt  # (N, 2)
        point2plane_dists = np.abs(np.sum(diff * normals, axis=1))  # |dot(diff, normal)|
        
        # Keep best 80% of matches
        keep = int(0.8 * len(point2plane_dists))
        idx_sort = np.argsort(point2plane_dists)
        sel = idx_sort[:keep]
        src = src[sel]
        tgt = tgt[sel]
        normals = normals[sel]
        point2plane_dists = point2plane_dists[sel]
        error = np.mean(point2plane_dists)
        
        # 3) Check convergence
        if abs(prevError - error) < ERROR_BOUND:
            success = True
            break
        
        prevError = error
        
        # 4) Solve for transformation using point-to-plane constraints
        # For 2D, we solve a linear system: A * [dx, dy, dtheta]^T = b
        # where each correspondence contributes one equation:
        # n · (R*p_src + t - p_tgt) = 0
        
        N = src.shape[0]
        A = np.zeros((N, 3))
        b = np.zeros(N)
        
        for i in range(N):
            nx, ny = normals[i]
            px, py = src[i]
            
            # Linearized rotation: R ≈ [1, -dtheta; dtheta, 1]
            # n · ([1, -dtheta; dtheta, 1] * p + [dx, dy] - p_tgt) = 0
            # n · ([px - dtheta*py + dx, py + dtheta*px + dy] - p_tgt) = 0
            # nx*(px - dtheta*py + dx) + ny*(py + dtheta*px + dy) - n·p_tgt = 0
            # nx*dx + ny*dy + dtheta*(-nx*py + ny*px) = n·p_tgt - n·p
            
            A[i, 0] = nx  # coefficient for dx
            A[i, 1] = ny  # coefficient for dy
            A[i, 2] = -nx * py + ny * px  # coefficient for dtheta
            
            b[i] = np.dot(normals[i], tgt[i] - src[i])
        
        # Solve least squares: A * x = b
        try:
            delta = np.linalg.lstsq(A, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            # Singular matrix, fall back to previous iteration
            success = False
            break
        
        dx, dy, dtheta = delta
        
        # Build incremental transformation
        c, s = np.cos(dtheta), np.sin(dtheta)
        R_inc = np.array([[c, -s], [s, c]])
        t_inc = np.array([dx, dy])
        
        # 5) Apply incremental transformation to current points
        curr_pts = (R_inc @ curr_pts.T).T + t_inc
        
        # 6) Accumulate total transformation
        R_total = R_inc @ R_total
        t_total = R_inc @ t_total + t_inc
    
    # Check if we reached max iterations without converging
    if count >= MAX_ITERATION:
        success = error < ERROR_BOUND
    
    # Extract final pose
    x = t_total[0]
    y = t_total[1]
    theta = np.arctan2(R_total[1, 0], R_total[0, 0])
    
    return x, y, theta, count, error, success

