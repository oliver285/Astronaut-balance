function vertices = tetrahedronVertices(a, b, c, d, e, f)
    % a: length between vertices 1 and 2
    % b: length between vertices 1 and 3
    % c: length between vertices 1 and 4
    % d: length between vertices 2 and 3
    % e: length between vertices 2 and 4
    % f: length between vertices 3 and 4

    % Coordinates of vertex 1
    A = [0, 0, 0];
    
    % Coordinates of vertex 2
    B = [a, 0, 0];

    % Calculate coordinates of vertex 3
    % Using the law of cosines to find position of C
    C_x = (b^2 + a^2 - d^2) / (2 * a);
    C_y = sqrt(b^2 - C_x^2);
    C = [C_x, C_y, 0];
    
    % Calculate coordinates of vertex 4
    % Using the law of cosines in 3D
    x4 = (c^2 + a^2 - e^2) / (2 * a);
    y4 = ((C_x^2 + C_y^2) + c^2 - f^2 - x4*2*C_x) / (2 * C_y);
    z4 = sqrt(c^2 - x4^2 - y4^2);
    D = [x4, y4, z4];

    % Combine all vertices into a matrix
    vertices = [A; B; C; D];
end