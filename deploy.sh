#!/bin/bash

# Настройки
PI_USER="robot"
PI_IP="10.42.0.1"
PI_DIR="~/RudeAnt" # Папка на малине, где лежит сишный код

echo "🚀 Синхронизация файлов на Малину..."

# Перекидываем код из src и include (исключая всякий мусор и папку build)
rsync -avz --exclude 'build/' --exclude '.git/' --exclude '__pycache__/' ./ $PI_USER@$PI_IP:$PI_DIR

echo "🛠 Компиляция на Малине..."
# Заходим по SSH, переходим в build и дергаем make
ssh $PI_USER@$PI_IP "cd $PI_DIR/build && make -j4"

echo "✅ Готово!"
