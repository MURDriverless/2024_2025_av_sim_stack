# Get 5 frames per video
all_sampled_frames = []  # This will hold 5 frames per video
all_sampled_labels = []          # 0 = real, 1 = fake

all_frames = []  # All frames from all videos
all_labels = []          # 0 = real, 1 = fake

for video_file in real_files:
    video_path = os.path.join(real_path, video_file)
    print(f"\nReading {video_file}...")

    cap = cv2.VideoCapture(video_path)
    all_frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        all_frames.append(frame)

    cap.release()
    total = len(all_frames)

    if total == 0:
        print("No frames found.")
        continue

    indices = np.linspace(0, total - 1, num=100, dtype=int)
    sampled = [all_frames[i] for i in indices]
    all_sampled_frames.extend(sampled)  # Add to master list
    all_labels.extend([0] * len(sampled))

    print(f"Stored {len(sampled)} sampled frames from {video_file}")

for video_file in fake_files:
    video_path = os.path.join(fake_path, video_file)
    print(f"\nReading {video_file}...")

    cap = cv2.VideoCapture(video_path)
    all_frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        all_frames.append(frame)

    cap.release()
    total = len(all_frames)

    if total == 0:
        print("No frames found.")
        continue

    indices = np.linspace(0, total - 1, num=100, dtype=int)
    sampled = [all_frames[i] for i in indices]
    all_sampled_frames.extend(sampled)  # Add to master list
    all_labels.extend([1] * len(sampled))

    print(f"Stored {len(sampled)} sampled frames from {video_file}")

print(f"\n✅ Total sampled frames collected: {len(all_sampled_frames)}")


# DONT RUN THISSSSSS
all_frames = []         # All frames from all videos
all_labels = []         # Corresponding labels (0 = real, 1 = fake)

sampled_frames = []     # 5 sampled frames per video
sampled_labels = []     # Labels for sampled frames

import numpy as np
import os
import cv2

def process_videos(video_list, label, folder_path):
    for video_file in video_list:
        video_path = os.path.join(folder_path, video_file)
        print(f"\nReading {video_file}...")

        cap = cv2.VideoCapture(video_path)
        video_frames = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            video_frames.append(frame)
            all_frames.append(frame)
            all_labels.append(label)

        cap.release()
        total = len(video_frames)

        if total == 0:
            print("No frames found.")
            continue

        # Sample 5 frames evenly across video
        indices = np.linspace(0, total - 1, num=5, dtype=int)
        selected = [video_frames[i] for i in indices]

        sampled_frames.extend(selected)
        sampled_labels.extend([label] * len(selected))

        print(f"Stored {total} total frames, and {len(selected)} sampled frames.")

# Process both real and fake
process_videos(real_files, label=0, folder_path=real_path)
process_videos(fake_files, label=1, folder_path=fake_path)

print(f"\n✅ Total all-frames collected: {len(all_frames)}")
print(f"✅ Total sampled frames collected: {len(sampled_frames)}")


from facenet_pytorch import MTCNN
from PIL import Image
import numpy as np
import torch
from torchvision import transforms
from torch.utils.data import TensorDataset

# Initialize MTCNN and transforms
mtcnn = MTCNN(keep_all=True, device=device)

transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
])

face_tensors = []
face_labels = []

# frames = all_sampled_frames
frames = all_frames

# Loop through each frame
for idx, frame in enumerate(frames):
    # sampled_labels = [inx]     # Labels for sampled frames
    label = all_labels[idx]  # 0 = real, 1 = fake

    # Convert frame
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)

    # Detect faces
    boxes, _ = mtcnn.detect(img_pil)

    if boxes is not None:
        faces = mtcnn.extract(img_pil, boxes, save_path=None)

        for face in faces:
            # Convert and transform
            face_img = face.permute(1, 2, 0).cpu().numpy()
            face_img = Image.fromarray(np.uint8((face_img + 1) * 127.5))
            face_tensor = transform(face_img)

            # Append data
            face_tensors.append(face_tensor)
            face_labels.append(label)
    else:
        print(f"Frame {idx}: No faces detected")

print(f"\n✅ Total face samples collected: {len(face_tensors)}")

# Create TensorDataset
dataset = TensorDataset(torch.stack(face_tensors), torch.tensor(face_labels))






from torch.utils.data import random_split

# Set sizes
total_len = len(dataset)
train_len = int(0.7 * total_len)
val_len = int(0.15 * total_len)
test_len = total_len - train_len - val_len  # remainder


# Random split
train_dataset, val_dataset, test_dataset = random_split(
    dataset, [train_len, val_len, test_len],
    generator=torch.Generator().manual_seed(42)  # reproducible split
)

# Define your augmentation pipeline
augmentation = transforms.Compose([
    transforms.RandomHorizontalFlip(p=0.5),
    transforms.RandomVerticalFlip(),
    transforms.RandomRotation(degrees=15),
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
])

# Step 1: Extract tensors and labels from train_dataset
train_images = []
train_labels = []

for x, y in train_dataset:
    # Convert to PIL Image, apply transform, convert back to tensor
    x_aug = augmentation(transforms.ToPILImage()(x))
    x_aug = transforms.ToTensor()(x_aug)
    train_images.append(x_aug)
    train_labels.append(y)

# Step 2: Stack into tensors
train_tensor = torch.stack(train_images)
label_tensor = torch.tensor(train_labels)

# Step 3: Create new augmented dataset
augmented_train_dataset = TensorDataset(train_tensor, label_tensor)

# Loaders
train_loader = DataLoader(augmented_train_dataset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=32)
test_loader = DataLoader(test_dataset, batch_size=32)






import os
from PIL import Image
from facenet_pytorch import MTCNN
import torch
from torchvision import transforms
from torch.utils.data import TensorDataset

# Mount Drive
drive.mount('/content/drive')

# Initialize MTCNN (same for both)
mtcnn = MTCNN(keep_all=True, image_size=160, device=device)

# Augmentations for training
train_transform = transforms.Compose([
    # transforms.Resize(224),
    transforms.RandomHorizontalFlip(),
    transforms.RandomRotation(10),
    transforms.GaussianBlur(kernel_size=3, sigma=1.0),
    transforms.ColorJitter(brightness=0.2, contrast=0.2),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225])
])

# Plain transforms for testing
test_transform = transforms.Compose([
    # transforms.Resize(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225])
])

def extract_faces_with_transform(root_path, transform):
    face_tensors = []

    for dirpath, _, filenames in os.walk(root_path):
        for file in filenames:
            if file.lower().endswith('.jpg'):
                img_path = os.path.join(dirpath, file)
                img = Image.open(img_path).convert('RGB')

                boxes, _ = mtcnn.detect(img)
                if boxes is not None:
                    faces = mtcnn.extract(img, boxes, save_path=None)
                    for face in faces:
                        face_img = face.permute(1, 2, 0).cpu().numpy()
                        face_img = Image.fromarray(np.uint8((face_img + 1) * 127.5))
                        face_tensor = transform(face_img)
                        face_tensors.append(face_tensor)

    return face_tensors

# Paths
train_path = '/content/drive/MyDrive/data/gpfs/projects/punim0811/Datasets/FFpp-c23-224/train'
test_path  = '/content/drive/MyDrive/data/gpfs/projects/punim0811/Datasets/FFpp-c23-224/test'

# train_path = '/data/gpfs/projects/punim0811/Datasets/FFpp-c23-224/train'
# test_path  = '/data/gpfs/projects/punim0811/Datasets/FFpp-c23-224/test'

# Extract with respective transforms
print("⏳ Extracting training faces...")
train_faces = extract_faces_with_transform(train_path, train_transform)
print(f"✅ Total train faces: {len(train_faces)}")

print("⏳ Extracting testing faces...")
test_faces = extract_faces_with_transform(test_path, test_transform)
print(f"✅ Total test faces: {len(test_faces)}")

# Create datasets
train_dataset = TensorDataset(torch.stack(train_faces))
test_dataset = TensorDataset(torch.stack(test_faces))





### For Testing only ###
# Random image URL
url = "https://plus.unsplash.com/premium_photo-1690407617542-2f210cf20d7e?q=80&w=2417&auto=format&fit=crop&ixlib=rb-4.1.0&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D"

# Download and open the image
response = requests.get(url)
img = Image.open(BytesIO(response.content))

# Display the image
plt.imshow(img)
plt.axis('off')
plt.show()








# Train Test Split - Extract Faces using MTCNN (Face detection Neural Network)

# Initialize MTCNN
mtcnn = MTCNN(keep_all=True)

# Detect faces
boxes, _ = mtcnn.detect(img)

if boxes is not None:
    print(f"Found {len(boxes)} faces.")
    # Extract faces and save them to the 'extracted_faces' folder in Colab
    faces = mtcnn.extract(img, boxes, save_path=None)

    # Debugging: Print some details of the extracted tensor
    print(f"Extracted {len(faces)} faces.")

    for i, face in enumerate(faces):
        print(f"Processing face {i}...")

        # Debug: Check tensor shape and values before conversion
        print(f"Face tensor shape: {face.shape}")
        print(f"Face tensor sample values: {face[0, 0, :5]}")  # First 5 values of the tensor

        # Denormalize the face tensor (range [-1, 1] to [0, 255]) --> could change but eh
        face_img = face.permute(1, 2, 0).float().cpu().numpy()  # Convert to float
        face_img = (face_img + 1) * 127.5  # Denormalize from [-1, 1] to [0, 255]
        face_img = np.clip(face_img, 0, 255)  # Ensure values are within [0, 255]

        # Convert to uint8 for image saving
        face_img = face_img.astype(np.uint8)

        # Debugging: Check the final NumPy array before saving
        print(f"NumPy array shape: {face_img.shape}")
        print(f"NumPy array sample values: {face_img[0, 0, :5]}")  # First 5 pixel values

        # Convert NumPy array to PIL image and save
        face_pil = Image.fromarray(face_img)
        # face_pil.save(f"/content/extracted_faces/face_{i}.jpg")
        plt.imshow(face_img)

        print(f"Saved face_{i}.jpg")
        ##### figure out how to save our data so we don't need to re-run all the time ###
else:
    print("No faces detected!")





# Mount Drive
drive.mount('/content/drive')

# Video folder
video_folder = '/content/drive/MyDrive/faceforensics/'
real_files = 'original_sequences/youtube/raw/videos/'
fake_files = 'manipulated_sequences/Deepfakes/raw/videos/'

real_path = os.path.join(video_folder, real_files)
fake_path = os.path.join(video_folder, fake_files)

real_files = [f for f in os.listdir(real_path) if f.endswith('.mp4')]
fake_files = [f for f in os.listdir(fake_path) if f.endswith('.mp4')]









# MOMENT CONTRAST LEARNING
from torch.utils.data import Dataset

class MoCoFaceDataset(Dataset):
    def __init__(self, face_tensors):
        self.face_tensors = face_tensors
        self.augment = transforms.Compose([
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(10),
            transforms.ColorJitter(brightness=0.2, contrast=0.2),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])

    def __len__(self):
        return len(self.face_tensors)

    def __getitem__(self, idx):
        img = self.face_tensors[idx]
        pil = transforms.ToPILImage()(img)
        return self.augment(pil), self.augment(pil)

import torch.nn as nn
import torch.nn.functional as F
import timm

class MoCoV3(nn.Module):
    def __init__(self, dim=256, K=4096, m=0.99, T=0.2):
        super().__init__()
        self.K = K
        self.m = m
        self.T = T

        self.encoder_q = timm.create_model("vit_small_patch16_224", pretrained=False, num_classes=0)
        self.encoder_k = timm.create_model("vit_small_patch16_224", pretrained=False, num_classes=0)

        self.projector_q = nn.Sequential(
            nn.Linear(self.encoder_q.num_features, dim),
            nn.ReLU(),
            nn.Linear(dim, dim)
        )
        self.projector_k = nn.Sequential(
            nn.Linear(self.encoder_k.num_features, dim),
            nn.ReLU(),
            nn.Linear(dim, dim)
        )

        for param_q, param_k in zip(self.encoder_q.parameters(), self.encoder_k.parameters()):
            param_k.data.copy_(param_q.data)
            param_k.requires_grad = False
        for param_q, param_k in zip(self.projector_q.parameters(), self.projector_k.parameters()):
            param_k.data.copy_(param_q.data)
            param_k.requires_grad = False

        self.register_buffer("queue", F.normalize(torch.randn(dim, K), dim=0))
        self.register_buffer("queue_ptr", torch.zeros(1, dtype=torch.long))

    @torch.no_grad()
    def momentum_update(self):
        for q, k in zip(self.encoder_q.parameters(), self.encoder_k.parameters()):
            k.data = self.m * k.data + (1. - self.m) * q.data
        for q, k in zip(self.projector_q.parameters(), self.projector_k.parameters()):
            k.data = self.m * k.data + (1. - self.m) * q.data

    @torch.no_grad()
    def enqueue_dequeue(self, keys):
        keys = keys.detach()
        batch_size = keys.shape[0]
        ptr = int(self.queue_ptr)
        self.queue[:, ptr:ptr + batch_size] = keys.T
        self.queue_ptr[0] = (ptr + batch_size) % self.K

    def forward(self, im_q, im_k):
        q = self.encoder_q(im_q)
        q = F.normalize(self.projector_q(q), dim=1)

        with torch.no_grad():
            self.momentum_update()
            k = self.encoder_k(im_k)
            k = F.normalize(self.projector_k(k), dim=1)

        l_pos = (q * k).sum(dim=1, keepdim=True)
        l_neg = torch.einsum('nc,ck->nk', [q, self.queue.clone().detach()])
        logits = torch.cat([l_pos, l_neg], dim=1) / self.T
        labels = torch.zeros(logits.size(0), dtype=torch.long).to(logits.device)

        self.enqueue_dequeue(k)
        return logits, labels
    

from torch.utils.data import DataLoader
import torch.optim as optim

# Initialize
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = MoCoV3().to(device)
criterion = nn.CrossEntropyLoss()
optimizer = optim.AdamW(model.parameters(), lr=1e-4)

# Training loop
for epoch in range(10):
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0

    for im_q, im_k in train_loader:
        im_q, im_k = im_q.to(device), im_k.to(device)
        logits, labels = model(im_q, im_k)

        loss = criterion(logits, labels)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        running_loss += loss.item()
        preds = torch.argmax(logits, dim=1)
        correct += (preds == labels).sum().item()
        total += labels.size(0)

    train_acc = 100 * correct / total
    train_loss = running_loss / len(train_loader)
    print(f"Epoch {epoch+1:02d} | Train Loss: {train_loss:.4f} | Train Acc: {train_acc:.2f}%")

    # Validation (test set)
    model.eval()
    val_loss = 0.0
    val_correct = 0
    val_total = 0

    with torch.no_grad():
        for im_q, im_k in val_loader:
            im_q, im_k = im_q.to(device), im_k.to(device)
            logits, labels = model(im_q, im_k)

            loss = criterion(logits, labels)
            val_loss += loss.item()

            preds = torch.argmax(logits, dim=1)
            val_correct += (preds == labels).sum().item()
            val_total += labels.size(0)

    val_acc = 100 * val_correct / val_total
    val_loss = val_loss / len(test_loader)
    print(f"           | Val Loss: {val_loss:.4f} | Val Acc: {val_acc:.2f}%\n")