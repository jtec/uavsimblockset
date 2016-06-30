function dcm = ffb_angle2dcm( a1, a2, a3)

angles = [a1(:) a2(:) a3(:)];

dcm = zeros(3,3,size(angles,1));
cosang = cos( angles );
sinang = sin( angles );

dcm(1,1,:) = cosang(:,2).*cosang(:,1);
dcm(1,2,:) = cosang(:,2).*sinang(:,1);
dcm(1,3,:) = -sinang(:,2);
dcm(2,1,:) = sinang(:,3).*sinang(:,2).*cosang(:,1) - cosang(:,3).*sinang(:,1);
dcm(2,2,:) = sinang(:,3).*sinang(:,2).*sinang(:,1) + cosang(:,3).*cosang(:,1);
dcm(2,3,:) = sinang(:,3).*cosang(:,2);
dcm(3,1,:) = cosang(:,3).*sinang(:,2).*cosang(:,1) + sinang(:,3).*sinang(:,1);
dcm(3,2,:) = cosang(:,3).*sinang(:,2).*sinang(:,1) - sinang(:,3).*cosang(:,1);
dcm(3,3,:) = cosang(:,3).*cosang(:,2);

end