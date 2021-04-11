# Overview

In this project the aim is to build a deep learning pipeline that can be deployed within a web or mobile-app to process user-supplied dog image to identify breed of the canine. 

## Step 1:Detect Human ##

In this task, **cv2.CascadeClassifier** has been used to detect human faces in an image. If the number of faces are 1 or biiger that means human has been detected in the image. 

/the cell to detect faces has been provided by Udacity:
```python
# returns "True" if face is detected in image stored at img_path
def face_detector(img_path):
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray)
    return len(faces) > 0
```

I just added the following to test the detector
```python
human_files_short = human_files[:100]
dog_files_short = dog_files[:100]

#-#-# Do NOT modify the code above this line. #-#-#

## TODO: Test the performance of the face_detector algorithm 
## on the images in human_files_short and dog_files_short.
human_counter=0
for human_file in human_files_short:
    if (face_detector(human_file)):
        human_counter += 1
not_human_counter=0
for dog_file in dog_files_short:
    if not (face_detector(dog_file)):
        not_human_counter += 1

print(f"Human counter: {human_counter}")
print(f"Not human counter: {not_human_counter}")
```

## Step 2: Making Predictions with a Pre-trained Model ##

In this task pre-trained VGG16 has been used to detect whether the given image contains dog or not.

1. Resize the image and convert to a tensor

```python
def load_image(img_path, max_size=224, shape=None):
    ''' Load in and transform an image, making sure the image
       has 224x224 pixels in the x-y dims.'''
    
    image = Image.open(img_path).convert('RGB')
    
    # large images will slow down processing
    if max(image.size) > max_size:
        size = max_size
    else:
        size = max(image.size)
    
    if shape is not None:
        size = shape
    
    # VGG16 specific input dimensions
    vgg_input_size= 224
    
    in_transform = transforms.Compose([
                        transforms.Resize([vgg_input_size,vgg_input_size]),
                        transforms.ToTensor(),
                        transforms.Normalize((0.485, 0.456, 0.406), 
                                             (0.229, 0.224, 0.225))])

    # discard the transparent, alpha channel (that's the :3) and add the batch dimension
    image = in_transform(image)[:3,:,:].unsqueeze(0)
    
    return image
```

2. Feed forward the tensor through the network and find the class has the biggest probability

```python
def VGG16_predict(img_path):
    '''
    Use pre-trained VGG-16 model to obtain index corresponding to 
    predicted ImageNet class for image at specified path
    
    Args:
        img_path: path to an image
        
    Returns:
        Index corresponding to VGG-16 model's prediction
    '''
    
    # Load the image as a tensor
    tensor = load_image(img_path).to("cpu")
    _, preds_tensor = torch.max(VGG16(tensor), 1)
    preds = np.squeeze(preds_tensor.cpu().numpy())
    
    return preds
```

## Step 3: Create a CNN to Classify Dog Breeds (from Scratch) ##

