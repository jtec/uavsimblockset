function dcm = ffb_quat2dcm( q )
qtn = quatnormalize( q );
dcm = zeros(3,3,size(qtn,1));

dcm(1,1,:) = qtn(:,1).^2 + qtn(:,2).^2 - qtn(:,3).^2 - qtn(:,4).^2;
dcm(1,2,:) = 2.*(qtn(:,2).*qtn(:,3) + qtn(:,1).*qtn(:,4));
dcm(1,3,:) = 2.*(qtn(:,2).*qtn(:,4) - qtn(:,1).*qtn(:,3));
dcm(2,1,:) = 2.*(qtn(:,2).*qtn(:,3) - qtn(:,1).*qtn(:,4));
dcm(2,2,:) = qtn(:,1).^2 - qtn(:,2).^2 + qtn(:,3).^2 - qtn(:,4).^2;
dcm(2,3,:) = 2.*(qtn(:,3).*qtn(:,4) + qtn(:,1).*qtn(:,2));
dcm(3,1,:) = 2.*(qtn(:,2).*qtn(:,4) + qtn(:,1).*qtn(:,3));
dcm(3,2,:) = 2.*(qtn(:,3).*qtn(:,4) - qtn(:,1).*qtn(:,2));
dcm(3,3,:) = qtn(:,1).^2 - qtn(:,2).^2 - qtn(:,3).^2 + qtn(:,4).^2;
